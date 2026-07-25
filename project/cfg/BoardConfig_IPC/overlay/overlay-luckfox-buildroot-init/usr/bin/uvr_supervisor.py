#!/usr/bin/env python3
"""
UVR SoC supervisor.

Runs on the RV1106 SoC, launched at boot. Responsibilities:
  - Keep the uvr-vicap capture running (relaunch only if it exits with an error;
    a clean exit means it finished its recording and we stay stopped).
  - Stream a heartbeat over UART0 reporting the current system state.
  - Parse the IMU frames the stm32 streams over UART0 and log them, alongside the
    SoC's internal temperature, to a CSV named to match the video file.
"""

import errno
import math
import os
import signal
import struct
import subprocess
import termios
import threading
import time
import traceback
import uuid
from datetime import datetime

# --- config (tweak here) ---------------------------------------------------
DATA_DIR             = "/data"                       # persistent output location
SERIAL_PORT          = "/dev/ttyS0"                  # UART0, shared link to the stm32
SERIAL_BAUD          = termios.B115200

SOC_TEMP_PATH        = "/sys/class/thermal/thermal_zone0/temp"  # millidegrees C
KMSG_PATH            = "/dev/kmsg"                   # kernel ring buffer

VICAP_BIN            = "/oem/usr/bin/uvr-vicap"
VICAP_FRAMES         = 1800                          # -l: frames per recording
VICAP_EXTRA_ARGS     = ["-v"]
VICAP_LOG            = "/data/uvr-vicap.log"

HEARTBEAT_INTERVAL_S = 0.2                           # UART0 state broadcast period
SOC_TEMP_EVERY       = 10                            # refresh SoC temp every N IMU rows
CSV_FLUSH_EVERY      = 100                           # flush CSV to disk every N rows
RESTART_DELAY_S      = 2.0                           # pause before relaunch after error
PROGRESS_EVERY_S     = 10                            # in-session progress log period
# ---------------------------------------------------------------------------

# Heartbeat state values, mirroring soc_status_t in the stm32 uart_format.h.
SOC_INIT      = 1
SOC_RECORDING = 2
SOC_STOPPED   = 3
SOC_ERROR     = 4

# IMU wire format: sync byte 0xAA followed by a packed imu_data_t
# (accel[3], gyro[3], temp), all little-endian floats.
IMU_SOF   = b"\xaa"
IMU_FMT   = "<7f"
IMU_LEN   = struct.calcsize(IMU_FMT)
FRAME_LEN = 1 + IMU_LEN

CSV_HEADER = ("timestamp,accel_x_g,accel_y_g,accel_z_g,"
              "gyro_x_dps,gyro_y_dps,gyro_z_dps,imu_temp_c,soc_temp_c")


def log(msg):
    os.write(2, ("[%s] %s\n" % (datetime.now().isoformat(), msg)).encode("ascii", "replace"))


def run_guarded(target, args):
    """Thread entry point that reports its own death instead of vanishing."""
    try:
        target(*args)
    except Exception:
        log("%s thread died:\n%s" % (target.__name__, traceback.format_exc()))


def retry_name(name, attempt):
    """video.h265 on the first attempt, video.1.h265 on the next, and so on."""
    if attempt == 0:
        return name
    stem, ext = os.path.splitext(name)
    return "%s.%d%s" % (stem, attempt, ext)


def make_run_dir():
    """Create this run's output directory and point DATA_DIR/latest at it."""
    name = "%s_%s" % (uuid.uuid4().hex[:12], datetime.now().strftime("%Y%m%dT%H%M%S"))
    path = os.path.join(DATA_DIR, name)
    os.makedirs(path, exist_ok=True)

    # Symlink last, via a rename, so `latest` is never left dangling.
    link, tmp = os.path.join(DATA_DIR, "latest"), os.path.join(DATA_DIR, "latest.new")
    try:
        os.symlink(name, tmp)
        os.replace(tmp, link)
    except OSError as e:
        log("could not update %s (%s)" % (link, e))
    return path


def open_output(path, preamble=b""):
    """Create a session output file, returning None if it can't be written.

    /data lives on an SD card that gets its power cut without warning, so a
    corrupt filesystem (EUCLEAN) or a full one is entirely possible. Losing a
    sidecar file is not a reason to lose the recording too.
    """
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        os.write(fd, preamble)
        return fd
    except OSError as e:
        log("cannot write %s (%s), continuing without it" % (path, e))
        return None


class Supervisor:
    def __init__(self):
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.state = SOC_INIT
        self.csv = None            # open file while a recording session is active
        self.klog = None           # ditto, for the kernel messages
        self.boot_log = b""        # ring buffer contents from before we started
        self.soc_temp = float("nan")
        self.row_count = 0
        self.temp_warned = False

    # --- sensors -----------------------------------------------------------
    def read_soc_temp(self):
        # os.read rather than open().read(): the buffered reader returns None
        # for this file (raw read gets EAGAIN), which breaks text decoding too.
        try:
            fd = os.open(SOC_TEMP_PATH, os.O_RDONLY)
            try:
                raw = os.read(fd, 32)
            finally:
                os.close(fd)
            return int(raw.strip().decode("ascii")) / 1000.0
        except (OSError, ValueError) as e:
            # Warn once: this is polled continuously, so logging every failure
            # would drown out everything else.
            if not self.temp_warned:
                self.temp_warned = True
                log("soc temp read failed (%s: %s), reporting nan from now on"
                    % (type(e).__name__, e))
            return float("nan")

    # --- IMU reception -----------------------------------------------------
    def on_imu(self, vals):
        with self.lock:
            fd = self.csv
            if fd is None:
                return
            self.row_count += 1
            if self.row_count == 1:
                log("first imu frame received")
            if self.row_count % SOC_TEMP_EVERY == 1:
                self.soc_temp = self.read_soc_temp()
            ax, ay, az, gx, gy, gz, itemp = vals
            try:
                os.write(fd, ("%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f\n" % (
                    datetime.now().isoformat(), ax, ay, az, gx, gy, gz, itemp,
                    self.soc_temp)).encode("ascii"))
                if self.row_count % CSV_FLUSH_EVERY == 0:
                    os.fsync(fd)
            except OSError as e:
                # Give up on the CSV rather than log this once per sample; the
                # recording itself is the more important artefact.
                log("csv write failed after %d rows (%s), abandoning csv"
                    % (self.row_count, e))
                self.csv = None

    def reader_loop(self, fd):
        buf = bytearray()
        while not self.stop.is_set():
            try:
                data = os.read(fd, 512)
            except OSError as e:
                log("serial read failed: %s" % e)
                self.stop.wait(0.1)
                continue
            if not data:
                continue
            buf.extend(data)
            while True:
                i = buf.find(IMU_SOF)
                if i < 0:
                    buf.clear()
                    break
                if i > 0:
                    del buf[:i]
                if len(buf) < FRAME_LEN:
                    break
                vals = struct.unpack(IMU_FMT, bytes(buf[1:FRAME_LEN]))
                if not all(math.isfinite(v) for v in vals):
                    # False sync byte inside float data: drop it and keep scanning.
                    del buf[0]
                    continue
                del buf[:FRAME_LEN]
                self.on_imu(vals)

    # --- kernel log --------------------------------------------------------
    def drain_kmsg(self, fd):
        """Read every record currently in the ring buffer, i.e. the boot log."""
        out = bytearray()
        while True:
            try:
                out += os.read(fd, 8192)
            except OSError as e:
                if e.errno == errno.EAGAIN:
                    return bytes(out)
                if e.errno != errno.EPIPE:
                    log("kmsg backlog read failed: %s" % e)
                    return bytes(out)

    def kmsg_loop(self, fd):
        while not self.stop.is_set():
            try:
                rec = os.read(fd, 8192)
            except OSError as e:
                if e.errno == errno.EAGAIN:
                    self.stop.wait(0.1)
                elif e.errno == errno.EPIPE:
                    # We fell behind and records were overwritten; the kernel has
                    # already moved us to the next available one.
                    log("kernel log overrun, some messages were lost")
                else:
                    log("kmsg read failed: %s" % e)
                    self.stop.wait(0.1)
                continue
            with self.lock:
                if self.klog is not None:
                    try:
                        os.write(self.klog, rec)
                    except OSError as e:
                        log("klog write failed (%s), abandoning kernel log" % e)
                        self.klog = None
        os.close(fd)

    # --- heartbeat ---------------------------------------------------------
    def heartbeat_loop(self, fd):
        while not self.stop.is_set():
            try:
                os.write(fd, bytes([self.state]))
            except OSError as e:
                log("heartbeat write failed: %s" % e)
            self.stop.wait(HEARTBEAT_INTERVAL_S)

    # --- recording ----------------------------------------------------------
    def record(self, run_dir, attempt):
        def out(name):
            return os.path.join(run_dir, retry_name(name, attempt))

        video = out("video.h265")

        csv_fd = open_output(out("imu.csv"), (CSV_HEADER + "\n").encode("ascii"))
        klog_fd = open_output(out("kernel.log"), self.boot_log)
        with self.lock:
            self.csv = csv_fd
            self.klog = klog_fd
            self.row_count = 0
            self.soc_temp = self.read_soc_temp()
        self.state = SOC_RECORDING

        argv = [VICAP_BIN, "-o", video, "-l", str(VICAP_FRAMES), *VICAP_EXTRA_ARGS]
        log("starting capture: %s" % " ".join(argv))

        vicap_log_fd = open_output(out("vicap.log"))
        try:
            proc = subprocess.Popen(
                argv,
                stdout=vicap_log_fd if vicap_log_fd is not None else subprocess.DEVNULL,
                stderr=subprocess.STDOUT)
            log("vicap running as pid %d" % proc.pid)
            rc = self.wait_for(proc, video)
        except OSError as e:
            log("failed to launch %s: %s" % (VICAP_BIN, e))
            rc = -1
        finally:
            if vicap_log_fd is not None:
                os.close(vicap_log_fd)

        with self.lock:
            self.csv = None
            self.klog = None
        for f in (csv_fd, klog_fd):
            if f is not None:
                os.close(f)
        log("capture ended: rc=%s, %d imu rows" % (rc, self.row_count))
        return rc

    def wait_for(self, proc, video):
        started = time.time()
        next_report = PROGRESS_EVERY_S
        while True:
            rc = proc.poll()
            if rc is not None:
                return rc
            elapsed = time.time() - started
            if elapsed >= next_report:
                next_report += PROGRESS_EVERY_S
                try:
                    size = os.stat(video).st_size
                except OSError:
                    size = 0
                log("  %ds elapsed: %d imu rows, %.1f MB video, soc %.1fC"
                    % (elapsed, self.row_count, size / 1e6, self.soc_temp))
            if self.stop.is_set():
                log("shutdown requested, terminating vicap")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return proc.poll()
            time.sleep(0.2)

    # --- main --------------------------------------------------------------
    def run(self):
        run_dir = make_run_dir()

        # Everything log() writes goes to stderr, so redirecting it here also
        # captures any traceback python prints on the way out.
        slog = open_output(os.path.join(run_dir, "supervisor.log"))
        if slog is not None:
            os.dup2(slog, 2)
            os.close(slog)

        log("supervisor starting (port=%s, output=%s)" % (SERIAL_PORT, run_dir))
        fd = open_serial(SERIAL_PORT)

        for handler in (signal.SIGTERM, signal.SIGINT):
            signal.signal(handler, lambda *_: self.stop.set())

        threads = [(self.reader_loop, (fd,)), (self.heartbeat_loop, (fd,))]

        # /dev/kmsg starts at the oldest surviving record, so the first drain
        # gives us the boot log; the tail thread then continues from exactly
        # where that stopped.
        try:
            kfd = os.open(KMSG_PATH, os.O_RDONLY | os.O_NONBLOCK)
            self.boot_log = self.drain_kmsg(kfd)
            log("captured %d bytes of kernel boot log" % len(self.boot_log))
            threads.append((self.kmsg_loop, (kfd,)))
        except OSError as e:
            log("cannot open %s (%s), no kernel log will be captured" % (KMSG_PATH, e))

        for target, args in threads:
            threading.Thread(target=run_guarded, args=(target, args), daemon=True).start()

        attempt = 0
        while not self.stop.is_set():
            try:
                rc = self.record(run_dir, attempt)
            except Exception:
                # Never let an unexpected failure take the supervisor down: the
                # stm32 would just see the heartbeat stop with no explanation.
                log("recording raised, treating as an error:\n" + traceback.format_exc())
                rc = -1
            if self.stop.is_set():
                break
            if rc == 0:
                self.state = SOC_STOPPED
                log("recording finished cleanly, state=STOPPED")
                break
            self.state = SOC_ERROR
            attempt += 1
            log("state=ERROR, retrying in %.1fs (attempt %d)" % (RESTART_DELAY_S, attempt))
            time.sleep(RESTART_DELAY_S)

        # Recording finished cleanly: keep heart-beating STOPPED (and draining the
        # UART) until we are told to shut down.
        while not self.stop.is_set():
            self.stop.wait(1.0)

        self.stop.set()
        os.close(fd)


def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag = oflag = lflag = 0                                   # raw
    cflag = (cflag & ~(termios.CSIZE | termios.PARENB)) | termios.CS8 | termios.CLOCAL | termios.CREAD
    ispeed = ospeed = SERIAL_BAUD
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 1                                       # 0.1s read timeout
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


if __name__ == "__main__":
    Supervisor().run()