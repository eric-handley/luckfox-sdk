#!/usr/bin/env python3
"""
UVR SoC supervisor.

Runs on the RV1106 SoC, launched at boot. Responsibilities:
  - Keep the uvr-vicap capture running (relaunch only if it exits with an error;
    a clean exit means it finished its recording and we stay stopped).
  - Stream a heartbeat over UART0 reporting the current system state.
  - Parse the IMU frames the stm32 streams over UART0 and log them, alongside the
    SoC's internal temperature, to a CSV.
  - Capture the kernel log, starting from the boot messages.

One run produces one directory, which DATA_DIR/latest points at:

    /data/1e37564f4f46_20210101T120247/
        supervisor.log      this file's output
        video.h265          the recording
        imu.csv             one row per IMU frame
        kernel.log          boot log, then everything logged while recording
        vicap.log           uvr-vicap's stdout/stderr

If a recording fails it is retried into the same directory, with the attempt
number inserted into each name (video.1.h265, imu.1.csv, ...) so nothing from
the failed attempt is lost.
"""

import errno
import math
import os
import signal
import struct
import subprocess
import sys
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
VICAP_MINUTES        = 5
VICAP_FRAMES         = VICAP_MINUTES * 60 * 30       # -l: frames per recording
VICAP_EXTRA_ARGS     = [""]

HEARTBEAT_INTERVAL_S = 0.2                           # UART0 state broadcast period
SOC_TEMP_EVERY       = 10                            # refresh SoC temp every N IMU rows
CSV_FLUSH_EVERY      = 100                           # flush CSV to disk every N rows
RESTART_DELAY_S      = 2.0                           # pause before relaunch after error
MAX_RECORD_ATTEMPTS  = 5                             # give up after this many failed tries
PROGRESS_EVERY_S     = 5                             # in-recording progress log period
SETTLE_AFTER_UNMOUNT_S = 5.0                         # let the SD card commit its cache before power-off
# ---------------------------------------------------------------------------

# Heartbeat state values, mirroring soc_status_t in the stm32 uart_format.h.
SOC_INIT      = 1
SOC_RECORDING = 2
SOC_STOPPED   = 3
SOC_ERROR     = 4
SOC_STOPPING  = 5   # imu stream stopped: saving the partial file and unmounting
SOC_COMPLETE  = 6   # the stm32 cuts our power on it; we repeat it until it does

# IMU wire format: sync byte 0xAA, then a packed imu_data_t (accel[3], gyro[3],
# temp), all little-endian floats.
IMU_SOF   = b"\xaa"
IMU_FMT   = "<7f"
IMU_LEN   = struct.calcsize(IMU_FMT)
FRAME_LEN = 1 + IMU_LEN

# The presence of the IMU stream is the record signal: if the stm32 is streaming
# frames we record, if none arrive in this window we are idle (booted only for
# servicing, e.g. USB pull) and exit without touching the camera.
IMU_WAIT_S = 3.0

# Once recording, the stm32 streams a frame every loop, so a gap this long means
# the stream stopped (FC requested a stop, or the link died): finalize and stop.
IMU_STOP_S = 5.0
# How long to let vicap flush and exit after SIGTERM before we SIGKILL it. It
# fflushes its write buffer and closes the file on the signal, so err generous to
# keep the partial recording.
VICAP_STOP_GRACE_S = 10.0

CSV_HEADER = ("timestamp,accel_x_g,accel_y_g,accel_z_g,"
              "gyro_x_dps,gyro_y_dps,gyro_z_dps,imu_temp_c,soc_temp_c")


def log(msg):
    os.write(2, ("[%s] %s\n" % (datetime.now().isoformat(), msg)).encode("ascii", "replace"))
    # Log lines are rare, so flushing every one costs nothing and means a power
    # cut can't take the explanation of what went wrong with it. Fails harmlessly
    # while stderr is still the init script's pipe.
    try:
        os.fsync(2)
    except OSError:
        pass


def klog(msg):
    """Mirror a milestone to /dev/kmsg so it lands on the serial console (and the
    captured kernel log), independent of the per-run supervisor.log on /data.
    The stop/finalize path unmounts /data and can be power-cut mid-way, taking
    its own log dir with it; this keeps the trail visible regardless."""
    try:
        fd = os.open("/dev/kmsg", os.O_WRONLY)
        os.write(fd, ("uvr: " + msg + "\n").encode("ascii", "replace"))
        os.close(fd)
    except OSError:
        pass


def _read_first(path):
    try:
        with open(path) as f:
            return f.readline()
    except OSError:
        return None


def load1():
    raw = _read_first("/proc/loadavg")
    try:
        return float(raw.split()[0])
    except (AttributeError, ValueError):
        return None


def mem_avail_mb():
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024.0   # kB -> MB
    except (OSError, ValueError):
        pass
    return None


def _fmt(v, spec):
    return (spec % v) if v is not None else "?"


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


def fsync_dir(path):
    """Persist a directory's entries. Without this a power cut orphans anything
    created in it since the last commit, since fsync on a file does not flush
    the parent directory that names it."""
    try:
        dfd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
    except OSError as e:
        log("could not fsync dir %s (%s)" % (path, e))


def make_run_dir():
    """Create this run's output directory and point DATA_DIR/latest at it."""
    name = "%s_%s" % (uuid.uuid4().hex[:12], datetime.now().strftime("%Y%m%dT%H%M%S"))
    path = os.path.join(DATA_DIR, name)
    os.makedirs(path, exist_ok=True)
    fsync_dir(DATA_DIR)

    # Symlink last, via a rename, so `latest` is never left dangling.
    link, tmp = os.path.join(DATA_DIR, "latest"), os.path.join(DATA_DIR, "latest.new")
    try:
        os.symlink(name, tmp)
        os.replace(tmp, link)
        fsync_dir(DATA_DIR)
    except OSError as e:
        log("could not update %s (%s)" % (link, e))

    # Append to a growing index so the recording order is recoverable even when
    # the clock is unset and the dir names don't sort chronologically.
    try:
        with open(os.path.join(DATA_DIR, "recordings.log"), "a") as f:
            f.write(name + "\n")
            f.flush()
            os.fsync(f.fileno())
    except OSError as e:
        log("could not append to recordings.log (%s)" % e)
    return path


def open_output(path, preamble=b""):
    """Create a run output file, returning None if it can't be written.

    /data lives on an SD card that gets its power cut without warning, so a
    corrupt filesystem (EUCLEAN) or a full one is entirely possible. Losing a
    sidecar file is not a reason to lose the recording too.
    """
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        os.write(fd, preamble)
        os.fsync(fd)
        return fd
    except OSError as e:
        log("cannot write %s (%s), continuing without it" % (path, e))
        return None


class Supervisor:
    def __init__(self, force_record=False):
        self.force_record = force_record
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.state = SOC_INIT
        self.csv = None            # open file while a recording is active
        self.klog = None           # ditto, for the kernel messages
        self.boot_log = b""        # ring buffer contents from before we started
        self.soc_temp = float("nan")
        self.row_count = 0
        self.temp_warned = False
        self.got_frame = threading.Event()
        self.last_frame = 0.0      # time.time() of the most recent imu frame
        self.graceful_stop = False # imu stream went quiet; finalize as a clean stop
        self.launch_failed = False # vicap binary missing / couldn't exec: don't retry

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
                self.got_frame.set()
                self.last_frame = time.time()
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
        # Warn once per failure episode: at 0.2s a dead port would otherwise spam
        # the log and bury the first, real failure. Re-arm on the next success.
        warned = False
        while not self.stop.is_set():
            try:
                os.write(fd, bytes([self.state]))
                warned = False
            except OSError as e:
                if not warned:
                    warned = True
                    log("heartbeat write failed: %s (suppressing until it recovers)" % e)
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
        self.last_frame = time.time()   # so the drought watchdog has a baseline
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
            # Persist all the run-dir entries (incl. video.h265, created by vicap)
            # so a power cut mid-recording keeps the partial files, not orphans.
            fsync_dir(run_dir)
            rc = self.wait_for(proc, video)
        except OSError as e:
            # Binary missing or not executable (e.g. /oem failed to mount). This
            # never fixes itself, so flag it as permanent: run() won't retry and
            # spray a video.N/imu.N/vicap.N set per doomed attempt.
            log("failed to launch %s: %s" % (VICAP_BIN, e))
            self.launch_failed = True
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
                log("  %ds elapsed: %d imu rows, %.1f MB video, soc %.1fC, "
                    "load %s, mem %s MB free"
                    % (elapsed, self.row_count, size / 1e6, self.soc_temp,
                       _fmt(load1(), "%.2f"), _fmt(mem_avail_mb(), "%.1f")))
            if self.stop.is_set():
                log("shutdown requested, terminating vicap")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return proc.poll()
            # In --record (manual ssh) mode there may be no stream at all, so the
            # drought check would fire instantly; skip it there.
            if not self.force_record and (time.time() - self.last_frame) > IMU_STOP_S:
                log("imu stream stopped for %.0fs, finalizing recording" % IMU_STOP_S)
                klog("drought: imu quiet %.0fs, starting graceful stop" % IMU_STOP_S)
                self.graceful_stop = True
                self.state = SOC_STOPPING
                proc.terminate()   # SIGTERM: vicap flushes and closes the partial
                try:
                    proc.wait(timeout=VICAP_STOP_GRACE_S)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return proc.poll()
            time.sleep(0.2)

    # --- main --------------------------------------------------------------
    def run(self):
        log("supervisor starting (port=%s)" % SERIAL_PORT)
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

        if self.force_record:
            log("--record: forcing record, ignoring imu presence")
        elif self.got_frame.wait(IMU_WAIT_S):
            log("imu stream present, recording")
        else:
            # No stream: this boot is idle, powered up only to service the card
            # (e.g. pull recordings over ssh). Remount /data read-only before we
            # get out of the way: a read-only fs is never dirtied, so the stm32 can
            # cut power at any moment -- planned or a surprise yank -- and the card
            # stays clean, with no journal recovery or e2fsck on the next boot. The
            # data is still fully readable over ssh.
            log("no imu after %.0fs, idle (not recording)" % IMU_WAIT_S)

            # Our stdout/stderr are the init script's >>/data/startup.log, which
            # holds /data open for writing and makes remount,ro fail EBUSY. Move
            # them to /dev/kmsg (closing the startup.log fd) first, exactly as the
            # finalize path does before it unmounts. log() keeps working, now on
            # the console/kernel log.
            os.sync()
            try:
                kfd = os.open("/dev/kmsg", os.O_WRONLY)
                os.dup2(kfd, 1)
                os.dup2(kfd, 2)
                os.close(kfd)
            except OSError:
                pass

            rc = subprocess.call(["mount", "-o", "remount,ro", DATA_DIR])
            if rc == 0:
                log("/data remounted read-only for idle")
                klog("idle: /data remounted read-only, safe to cut power")
            else:
                log("could not remount /data read-only (rc=%d)" % rc)
                klog("idle: FAILED to remount /data read-only (rc=%d)" % rc)
            return

        # Only create a run directory once we know we are recording, so idle mode
        # leaves DATA_DIR/latest pointing at the last real recording.
        run_dir = make_run_dir()

        # Everything log() writes goes to stderr, so redirecting it here also
        # captures any traceback python prints on the way out. Startup lines up
        # to this point stay in the init script's log.
        slog = open_output(os.path.join(run_dir, "supervisor.log"))
        if slog is not None:
            os.dup2(slog, 2)
            os.close(slog)
        log("output=%s" % run_dir)

        rc = -1
        attempt = 0
        while not self.stop.is_set():
            try:
                rc = self.record(run_dir, attempt)
            except Exception:
                # This is our own bug (bad format string, TypeError, ...), not a
                # vicap failure. Don't retry it forever into video.1, video.2, ...;
                # surface it and stop. state=ERROR keeps the heartbeat meaningful
                # so the stm32 still sees a reason rather than silence.
                log("record() raised, aborting:\n" + traceback.format_exc())
                self.state = SOC_ERROR
                break
            if self.stop.is_set() or self.graceful_stop:
                break
            if rc == 0:
                log("recording finished cleanly")
                break
            self.state = SOC_ERROR
            if self.launch_failed:
                # Permanent (missing/!exec binary): retrying just spews files.
                log("vicap launch failed permanently, giving up")
                break
            attempt += 1
            if attempt >= MAX_RECORD_ATTEMPTS:
                log("giving up after %d failed attempts" % attempt)
                break
            log("state=ERROR, retrying in %.1fs (attempt %d/%d)"
                % (RESTART_DELAY_S, attempt, MAX_RECORD_ATTEMPTS))
            time.sleep(RESTART_DELAY_S)

        # Stop the reader/kmsg/heartbeat threads so nothing else touches the
        # port or the output files, then drive the final state ourselves.
        self.stop.set()

        # A clean frame-limit finish (rc==0) and a graceful stop (imu stream went
        # quiet, partial already saved by vicap on SIGTERM) both mean "recording
        # is safely on disk, cut power". Anything else is a genuine failure.
        done = (rc == 0) or self.graceful_stop

        if self.force_record:
            # Stay powered so an ssh session survives; leave /data mounted.
            if done:
                log("syncing to disk")
                os.sync()
            self.state = SOC_STOPPED
            try:
                os.write(fd, bytes([self.state]))
            except OSError:
                pass
            os.close(fd)
            return

        if not done:
            # Gave up on a real error. Report it and let the stm32's heartbeat
            # timeout deal with power rather than unmounting/completing.
            klog("finalize: NOT done (rc=%s graceful=%s) -> SOC_ERROR, no unmount"
                 % (rc, self.graceful_stop))
            self.state = SOC_ERROR
            try:
                os.write(fd, bytes([self.state]))
            except OSError:
                pass
            os.close(fd)
            return

        # Tell the stm32 we're finalizing, then leave /data cleanly unmounted.
        klog("finalize: done, SOC_STOPPING, about to sync /data")
        self.state = SOC_STOPPING
        try:
            os.write(fd, bytes([self.state]))
        except OSError:
            pass
        log("syncing and unmounting /data before power-off")
        os.sync()
        klog("finalize: sync() returned, unmounting /data")

        # Our stdout/stderr (incl. the supervisor.log fd) live on /data and would
        # hold it busy, and so would our cwd if it's anywhere under /data. Move
        # both off it first. Logging goes to /dev/kmsg rather than /dev/null so
        # the umount result stays visible on the console we're capturing.
        try:
            os.chdir("/")
        except OSError:
            pass
        try:
            kfd = os.open("/dev/kmsg", os.O_WRONLY)
            os.dup2(kfd, 1)
            os.dup2(kfd, 2)
            os.close(kfd)
        except OSError:
            pass

        # Retry: right after reaping vicap the mount can briefly report EBUSY.
        umounted = False
        for attempt in range(5):
            rc = subprocess.call(["umount", DATA_DIR])
            if rc == 0:
                umounted = True
                break
            log("umount /data failed (rc=%d), retry %d/5" % (rc, attempt + 1))
            time.sleep(0.5)
        if umounted:
            log("/data unmounted cleanly")
            klog("finalize: /data unmounted cleanly")
        else:
            # Something still holds /data open (e.g. vicap wedged in the kernel).
            # Do NOT leave it mounted rw and merely synced: the fs stays dirty,
            # background writeback/jbd2 can re-touch metadata, and the imminent
            # power cut then loses bitmap updates -> next-boot e2fsck fixes. A
            # remount,ro succeeds even with files open: it flushes the journal and
            # stops all further writes, so the cut lands on a quiesced fs.
            rc = subprocess.call(["mount", "-o", "remount,ro", DATA_DIR])
            if rc == 0:
                log("could not unmount /data; remounted read-only instead")
                klog("finalize: /data remounted read-only (umount busy)")
            else:
                log("could not unmount or remount-ro /data; syncing only")
                klog("finalize: /data NOT quiesced, sync only (will need fsck)")
                os.sync()

        os.sync()
        time.sleep(SETTLE_AFTER_UNMOUNT_S)

        # COMPLETE tells the stm32 to cut our power. Repeat it on the heartbeat
        # interval until it does, so a single dropped byte doesn't strand us
        # powered; the loop ends when the rails are pulled out from under us.
        klog("finalize: sending SOC_COMPLETE until power is cut")
        self.state = SOC_COMPLETE
        while True:
            try:
                os.write(fd, bytes([self.state]))
            except OSError:
                pass
            time.sleep(HEARTBEAT_INTERVAL_S)


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
    Supervisor(force_record="--record" in sys.argv[1:]).run()