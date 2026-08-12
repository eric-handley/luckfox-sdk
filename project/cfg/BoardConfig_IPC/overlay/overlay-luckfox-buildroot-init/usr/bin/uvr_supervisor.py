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
        audio.wav           mic capture (tinycap), started with the video
        imu.csv             one row per IMU frame
        kernel.log          boot log, then everything logged while recording
        vicap.log           uvr-vicap's stdout/stderr
        audio.log           tinycap's stdout/stderr

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
VICAP_MINUTES        = 30
VIDEO_FPS            = 60                             # sensor/ISP output cadence
VICAP_FRAMES         = VICAP_MINUTES * 60 * VIDEO_FPS # -l: frames per recording
VICAP_EXTRA_ARGS     = [""]

AUDIO_BIN            = "/usr/bin/tinycap"            # tinyalsa capture, WAV out
AUDIO_CARD           = 0                             # ALSA card (rv1106-acodec)
AUDIO_DEVICE         = 0                             # PCM device on that card
AUDIO_CHANNELS       = 2                             # acodec capture DAI is stereo-only;
                                                     # a 1ch open is rejected EINVAL. The
                                                     # mic lands on the LEFT ADC (ch0);
                                                     # downmix mono host-side in the mux (c0).
AUDIO_RATE           = 48000                         # sample rate (Hz)
AUDIO_BITS           = 16                            # sample width (bits)

AUDIO_MIXER_BIN      = "/usr/bin/tinymix"            # codec control (tinyalsa)
# Codec front-end config applied before each capture. The right-channel gains +
# MICBIAS on, at the codec's power-on default ADC Mode, are what put a working
# mic on ch0/LEFT. Do NOT add an "ADC Mode" write here -- moving ADC Mode off the
# boot default breaks the input and the mic no longer lands cleanly on ch0.
# The mic is on ch0/LEFT, so the left-channel gains (previously left at boot
# defaults) are the ones that actually amplify it; max them and turn on left AGC
# for more level on quiet input.
AUDIO_MIXER_SETTINGS = [
    ("ADC MIC Right Gain",       "2"),
    ("ADC ALC Right Volume",     "31"),
    ("ADC Digital Right Volume", "255"),
    ("ADC Main MICBIAS",         "1"),
    ("ADC MIC Left Gain",        "2"),
    ("ADC ALC Left Volume",      "31"),
    ("ADC Digital Left Volume",  "204"),   # 80% of full-scale digital gain
    ("AGC Left Approximate Sample Rate", "1"),   # 48 kHz, match the capture rate
    ("ALC AGC Left Switch",      "1"),           # AGC on (left channel)
]

HEARTBEAT_INTERVAL_S = 0.2                           # UART0 state broadcast period
SOC_TEMP_EVERY       = 10                            # refresh SoC temp every N IMU rows
CSV_FLUSH_EVERY      = 100                           # flush CSV to disk every N rows
RESTART_DELAY_S      = 2.0                           # pause before relaunch after error
MAX_RECORD_ATTEMPTS  = 5                             # give up after this many failed tries
CSV_OPEN_ATTEMPTS    = 5                             # retries opening the imu csv before giving up
CSV_OPEN_RETRY_S     = 0.5                           # pause between those csv-open retries
CSV_REOPEN_MAX       = 5                             # times to reopen the csv after a write failure per run
READER_MAX_RESTARTS  = 5                             # relaunches of a crashed imu reader before giving up
READER_RESTART_DELAY_S = 0.5                         # pause before relaunching a crashed reader
PROGRESS_EVERY_S     = 5                             # in-recording progress log period
SETTLE_AFTER_UNMOUNT_S = 5.0                         # let the SD card commit its cache before power-off
CMD_DEBOUNCE_N       = 3                             # consecutive identical frame commands before we act
READER_STATS_EVERY_S = 5.0                           # period for the reader's reject/garbage summary

KERN_PRINTK_PATH     = "/proc/sys/kernel/printk"     # kernel console log-level control
KERN_CONSOLE_LOGLEVEL = 3                            # keep emerg/alert/crit on console, drop err/warn/info
KLOG_LEVEL           = KERN_CONSOLE_LOGLEVEL - 1     # priority for our klog() lines: below the console
                                                     #   threshold so they still print on the serial tty
                                                     #   after we lower it (else our own milestones vanish
                                                     #   along with the err/warn spam we're muting)
KLOG_WINDOW_S        = 1.0                           # rate-limit window for kmsg -> kernel.log writes
KLOG_HOT_PER_WINDOW  = 50                            # records/window that marks a window "hot" (steady-state
                                                     #   recording is <=17/s; camera-init blips to ~80/s)
KLOG_OVERRUN_HOT     = 2                             # ...or this many ring-buffer overruns/window (we're
                                                     #   losing records = flooding faster than we can read)
KLOG_HOT_SUSTAIN     = 3                             # consecutive hot windows before we start dropping, so a
                                                     #   brief camera-init burst never trips it, only a storm
# ---------------------------------------------------------------------------

# Heartbeat state values, mirroring soc_status_t in the stm32 uart_format.h.
SOC_INIT      = 1
SOC_RECORDING = 2
SOC_STOPPED   = 3
SOC_ERROR     = 4
SOC_STOPPING  = 5   # stop requested: saving the partial file and unmounting
SOC_COMPLETE  = 6   # the stm32 cuts our power on it; we repeat it until it does
SOC_IDLE      = 7   # idle: /data read-only, parked; repeated so the stm32 sees it

# Per-frame command tags, mirroring frame_cmd_t in the stm32 uart_format.h.
FRAME_CMD_RECORD = 1
FRAME_CMD_IDLE   = 2
FRAME_CMD_STOP   = 3
FRAME_CMDS       = frozenset((FRAME_CMD_RECORD, FRAME_CMD_IDLE, FRAME_CMD_STOP))
CMD_NAMES        = {FRAME_CMD_RECORD: "RECORD", FRAME_CMD_IDLE: "IDLE",
                    FRAME_CMD_STOP: "STOP", None: "none (default record)"}

# IMU wire format: sync byte 0xAA, a command byte (frame_cmd_t), then a packed
# imu_data_t (accel[3], gyro[3], temp), all little-endian floats.
IMU_SOF   = b"\xaa"
IMU_FMT   = "<7f"
IMU_LEN   = struct.calcsize(IMU_FMT)
FRAME_LEN = 2 + IMU_LEN

# How long to wait at boot for the first debounced command before defaulting to
# RECORD. A silent or garbled link therefore records rather than idling.
COMMAND_WAIT_S = 3.0
# How long to let vicap flush and exit after SIGTERM before we SIGKILL it. It
# fflushes its write buffer and closes the file on the signal, so err generous to
# keep the partial recording.
VICAP_STOP_GRACE_S = 10.0
# Cap tinycap's own run length a little past the video's frame budget, so the
# video (frame-limited) or a CMD_STOP always ends the pair first; the -t is just
# a backstop against a wedged stop leaving it recording forever.
AUDIO_MAX_S        = VICAP_MINUTES * 60 + 30
# Let tinycap flush and rewrite the WAV header after SIGINT before we SIGKILL it.
AUDIO_STOP_GRACE_S = 5.0

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
    its own log dir with it; this keeps the trail visible regardless.

    Tagged <KLOG_LEVEL> (below KERN_CONSOLE_LOGLEVEL) so it still prints to the
    serial console after quiet_kernel_console() lowers the threshold -- otherwise
    our own milestones would get filtered out with the err/warn spam we mute."""
    try:
        fd = os.open("/dev/kmsg", os.O_WRONLY)
        os.write(fd, ("<%d>uvr: %s\n" % (KLOG_LEVEL, msg)).encode("ascii", "replace"))
        os.close(fd)
    except OSError:
        pass


def quiet_kernel_console(level=KERN_CONSOLE_LOGLEVEL):
    """Lower the kernel console log level so a flood of low-priority kernel
    messages can't stall the CPU. A marginal camera link throws thousands of
    KERN_ERR mipi/csi errors per second; each one is printed synchronously to the
    115200 serial console (console=ttyS2), which burns the single core and starves
    our reader so a STOP never gets serviced. Dropping err/warn/info from the
    console spares that cost. The messages still reach the ring buffer, so
    /dev/kmsg and kernel.log keep capturing them"""
    try:
        fd = os.open(KERN_PRINTK_PATH, os.O_WRONLY)
        try:
            os.write(fd, ("%d\n" % level).encode("ascii"))
        finally:
            os.close(fd)
        log("kernel console loglevel set to %d" % level)
        # Echo it on the serial console too, so a live session can confirm the
        # mitigation is armed before reproducing the fault.
        klog("kernel console loglevel set to %d (err/warn/info muted on console)"
             % level)
    except OSError as e:
        log("could not set kernel console loglevel (%s)" % e)


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
    name = uuid.uuid4().hex[:12]
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


def move_stdio_to_kmsg():
    """Redirect fd 1/2 to /dev/kmsg, closing whatever they pointed at before
    (e.g. a file on /data). Used before remounting/unmounting /data so our own
    stdio doesn't hold it busy; log() keeps working, now on the console."""
    try:
        kfd = os.open("/dev/kmsg", os.O_WRONLY)
        os.dup2(kfd, 1)
        os.dup2(kfd, 2)
        os.close(kfd)
    except OSError as e:
        # Not fatal, but it means our stdio may still pin /data, so a following
        # remount/unmount can fail EBUSY -- record it (klog, since stdio is the
        # thing that just failed) rather than let it vanish.
        klog("could not redirect stdio to kmsg (%s); /data may stay busy" % e)


def close_fds(*fds):
    for fd in fds:
        if fd is not None:
            os.close(fd)


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


def open_output_retry(path, preamble=b""):
    """open_output that retries a transient failure a few times before giving up.

    A momentary open failure (the fs briefly busy/remounting) would otherwise
    disable a sidecar file for the whole run; a bounded retry lets it recover.
    Still returns None (never fatal) if every attempt fails.
    """
    for attempt in range(CSV_OPEN_ATTEMPTS):
        fd = open_output(path, preamble)
        if fd is not None:
            return fd
        if attempt < CSV_OPEN_ATTEMPTS - 1:
            time.sleep(CSV_OPEN_RETRY_S)
    log("gave up opening %s after %d attempts, continuing without it"
        % (path, CSV_OPEN_ATTEMPTS))
    return None


class Supervisor:
    def __init__(self, force_record=False):
        self.force_record = force_record
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.state = SOC_INIT
        self.csv = None            # open file while a recording is active
        self.csv_path = None       # its path, so a failed write can reopen it
        self.csv_reopen_left = 0   # remaining reopen attempts this recording
        self.klog = None           # ditto, for the kernel messages
        self.boot_log = b""        # ring buffer contents from before we started
        self.soc_temp = float("nan")
        self.row_count = 0
        self.temp_warned = False
        self.command = None        # latest debounced frame command (FRAME_CMD_*)
        self.command_seen = threading.Event()  # set once the first command commits
        self._pending_cmd = None   # command awaiting debounce confirmation
        self._pending_count = 0    # consecutive frames carrying _pending_cmd
        self.graceful_stop = False # got CMD_STOP: finalize as a clean stop
        self.launch_failed = False # vicap binary missing / couldn't exec: don't retry

    def set_state(self, state):
        with self.lock:
            self.state = state

    def send_state(self, fd, context=None):
        """Write the current state to the uart, snapshotting it under the same
        lock as set_state() so a concurrent transition can't be torn. A write
        failure is silent for the repeated heartbeat, but the one-shot finalize
        transitions pass a context so a lost hand-off to the stm32 is logged."""
        with self.lock:
            b = bytes([self.state])
        try:
            os.write(fd, b)
        except OSError as e:
            if context is not None:
                log("failed to send %s to stm32 (%s)" % (context, e))

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
    def reopen_csv(self):
        """Reopen the imu csv (append, preserving rows already written) after a
        write failure, so one transient error doesn't drop the csv for the whole
        recording. Bounded by CSV_REOPEN_MAX so a persistently broken fs doesn't
        turn every row into an open attempt. Caller holds self.lock."""
        if self.csv_path is None or self.csv_reopen_left <= 0:
            return None
        self.csv_reopen_left -= 1
        try:
            fd = os.open(self.csv_path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            log("reopened %s after write failure (%d reopen attempts left)"
                % (self.csv_path, self.csv_reopen_left))
            return fd
        except OSError as e:
            log("could not reopen %s (%s)" % (self.csv_path, e))
            return None

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
                # Try to recover by reopening (append) rather than dropping the
                # csv for the rest of the run on one write error. Only once the
                # bounded reopen attempts are spent do we give up; the recording
                # itself is the more important artefact.
                log("csv write failed after %d rows (%s), reopening"
                    % (self.row_count, e))
                try:
                    os.close(fd)
                except OSError:
                    pass
                self.csv = self.reopen_csv()
                if self.csv is None:
                    log("csv reopen exhausted, abandoning csv")

    def observe_command(self, cmd):
        """Debounce the per-frame command: only commit (and set command_seen)
        after CMD_DEBOUNCE_N consecutive identical commands, so a single flipped
        byte can't switch mode. Called from the reader thread only."""
        if cmd == self._pending_cmd:
            self._pending_count += 1
        else:
            self._pending_cmd = cmd
            self._pending_count = 1
        if self._pending_count >= CMD_DEBOUNCE_N and cmd != self.command:
            self.command = cmd
            self.command_seen.set()

    def reader_manager(self, fd):
        """Keep an IMU reader alive for the whole run. reader_loop only returns on
        shutdown; if it returns otherwise it hit an unexpected fault, so flush the
        uart and relaunch it (bounded) rather than leaving the reader dead."""
        restarts = 0
        while not self.stop.is_set():
            try:
                self.reader_loop(fd)
            except Exception:
                log("imu reader crashed:\n%s" % traceback.format_exc())
            if self.stop.is_set():
                break
            restarts += 1
            if restarts > READER_MAX_RESTARTS:
                log("imu reader failed %d times, giving up on imu capture"
                    % READER_MAX_RESTARTS)
                return
            log("imu reader down, reinitializing uart and restarting (%d/%d)"
                % (restarts, READER_MAX_RESTARTS))
            try:
                termios.tcflush(fd, termios.TCIFLUSH)
            except OSError as e:
                log("uart flush during reader recovery failed: %s" % e)
            self.stop.wait(READER_RESTART_DELAY_S)

    def reader_loop(self, fd):
        buf = bytearray()
        # Reception health, summarized periodically so sustained corruption or
        # misalignment is visible instead of silently masquerading as "no data".
        ok = rejected = garbage = 0
        last_stats = time.time()
        while not self.stop.is_set():
            try:
                data = os.read(fd, 512)
            except OSError as e:
                log("serial read failed: %s" % e)
                self.stop.wait(0.1)
                continue
            if data:
                buf.extend(data)
                while True:
                    i = buf.find(IMU_SOF)
                    if i < 0:
                        garbage += len(buf)   # no sync byte anywhere: all junk
                        buf.clear()
                        break
                    if i > 0:
                        garbage += i          # junk before the sync byte
                        del buf[:i]
                    if len(buf) < FRAME_LEN:
                        break
                    cmd = buf[1]
                    vals = struct.unpack(IMU_FMT, bytes(buf[2:FRAME_LEN]))
                    if cmd not in FRAME_CMDS or not all(math.isfinite(v) for v in vals):
                        # False sync byte or corrupt frame: drop it and rescan.
                        rejected += 1
                        del buf[0]
                        continue
                    ok += 1
                    self.observe_command(cmd)
                    del buf[:FRAME_LEN]
                    self.on_imu(vals)
            now = time.time()
            if now - last_stats >= READER_STATS_EVERY_S:
                if rejected or garbage:
                    log("imu reader: %d frames ok, %d rejected, %d garbage bytes "
                        "in last %.0fs" % (ok, rejected, garbage, now - last_stats))
                ok = rejected = garbage = 0
                last_stats = now

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

    def _klog_write(self, data):
        """Append raw bytes to the run's kernel.log, dropping it on write error."""
        with self.lock:
            if self.klog is not None:
                try:
                    os.write(self.klog, data)
                except OSError as e:
                    log("klog write failed (%s), abandoning kernel log" % e)
                    self.klog = None

    def kmsg_loop(self, fd):
        # Normally we mirror every kernel record into kernel.log. But a degraded
        # camera link can flood the ring buffer far faster than we can drain it
        # (thousands/sec), which burns CPU + SD IO, balloons the file, and shows
        # up as EPIPE overruns as records get overwritten before we read them.
        # quiet_kernel_console() spares the big cost (those going to the serial
        # console); this spares our own. When a window is "hot" -- either the
        # record rate crosses KLOG_HOT_PER_WINDOW or we take KLOG_OVERRUN_HOT+
        # overruns (real rate is masked by the loss) -- for KLOG_HOT_SUSTAIN
        # windows running, we stop writing records and emit one summary per window
        # instead. The sustain requirement means only a real storm trips it; the
        # brief camera-init burst (~80/s for 1-2s) rides through untouched. A few
        # lost ISP records don't matter -- the detail is in vicap.log.
        win_start = time.time()
        seen = dropped = overruns = 0
        hot_streak = 0
        engaged = False
        while not self.stop.is_set():
            try:
                rec = os.read(fd, 8192)
            except OSError as e:
                if e.errno == errno.EAGAIN:
                    self.stop.wait(0.1)
                elif e.errno == errno.EPIPE:
                    # We fell behind and records were overwritten; the kernel has
                    # moved us to the next available one. Counted per window (a
                    # storm signal in its own right), not logged per event.
                    overruns += 1
                else:
                    log("kmsg read failed: %s" % e)
                    self.stop.wait(0.1)
                # Fall through so the window still rolls over even under an EPIPE
                # storm (where os.read never returns a record).
                rec = None
            else:
                seen += 1
                if engaged:
                    dropped += 1
                else:
                    self._klog_write(rec)

            now = time.time()
            if now - win_start >= KLOG_WINDOW_S:
                span = now - win_start
                hot = seen >= KLOG_HOT_PER_WINDOW or overruns >= KLOG_OVERRUN_HOT
                hot_streak = hot_streak + 1 if hot else 0
                was = engaged
                engaged = hot_streak >= KLOG_HOT_SUSTAIN
                if engaged:
                    msg = ("kernel log flooding: %d records/%.0fs (dropped %d, "
                           "%d overruns)" % (seen, span, dropped, overruns))
                    self._klog_write(("uvr: " + msg + "\n").encode("ascii", "replace"))
                    log(msg)
                    klog(msg)          # to the serial console, for live verification
                elif was:
                    log("kernel log flood subsided")
                    klog("kernel log flood subsided")
                elif overruns:
                    # Not (yet) a sustained storm, but we still lost records --
                    # note it once for the window rather than per event.
                    log("kernel log overrun, %d records lost" % overruns)
                win_start = now
                seen = dropped = overruns = 0
        os.close(fd)

    # --- heartbeat ---------------------------------------------------------
    def heartbeat_loop(self, fd):
        # Warn once per failure episode: at 0.2s a dead port would otherwise spam
        # the log and bury the first, real failure. Re-arm on the next success.
        warned = False
        while not self.stop.is_set():
            with self.lock:
                b = bytes([self.state])
            try:
                os.write(fd, b)
                warned = False
            except OSError as e:
                if not warned:
                    warned = True
                    log("heartbeat write failed: %s (suppressing until it recovers)" % e)
            self.stop.wait(HEARTBEAT_INTERVAL_S)

    # --- audio ---------------------------------------------------------------
    def configure_codec(self):
        """Apply the codec front-end settings the mic needs (see
        AUDIO_MIXER_SETTINGS). Best-effort: a failure here never blocks the
        recording, same as the rest of the audio path."""
        for name, value in AUDIO_MIXER_SETTINGS:
            try:
                rc = subprocess.call(
                    [AUDIO_MIXER_BIN, "set", name, value],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if rc != 0:
                    log("tinymix set %r=%s returned %d" % (name, value, rc))
            except OSError as e:
                log("failed to run tinymix (%s); skipping codec config" % e)
                return

    def start_audio(self, path):
        """Best-effort mic capture (tinycap -> WAV), started alongside vicap so
        the two begin together. Audio is subordinate to the video: a failure to
        launch or record never fails the recording, mirroring the csv sidecar.
        Returns (proc, log_fd), either of which may be None."""
        self.configure_codec()
        log_fd = open_output(os.path.splitext(path)[0] + ".log")  # audio.wav -> audio.log
        argv = [AUDIO_BIN, path,
                "-D", str(AUDIO_CARD), "-d", str(AUDIO_DEVICE),
                "-c", str(AUDIO_CHANNELS), "-r", str(AUDIO_RATE),
                "-b", str(AUDIO_BITS), "-t", str(AUDIO_MAX_S)]
        try:
            proc = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,   # tinycap ignores stdin; keep it off the terminal
                stdout=log_fd if log_fd is not None else subprocess.DEVNULL,
                stderr=subprocess.STDOUT)
            # A/V sync anchor: monotonic time tinycap was launched. Its first
            # captured sample follows within the ALSA-open latency (~ms), unlike
            # vicap whose frames start seconds after launch (camera init) -- so
            # vicap stamps its first frame from inside, we stamp launch here.
            # Same distinctive token in both logs; the host mux aligns by it.
            if log_fd is not None:
                try:
                    us = int(time.clock_gettime(time.CLOCK_MONOTONIC) * 1_000_000)
                    os.write(log_fd, ("SYNC_START_US=%d\n" % us).encode("ascii"))
                except OSError:
                    pass
            log("audio capture running as pid %d: %s" % (proc.pid, " ".join(argv)))
            return proc, log_fd
        except OSError as e:
            log("failed to launch audio capture %s: %s (continuing without audio)"
                % (AUDIO_BIN, e))
            close_fds(log_fd)
            return None, None

    def stop_audio(self, proc, log_fd):
        """Stop tinycap with SIGINT so it finalizes the WAV header (rewrites the
        length), bounded by a grace before we SIGKILL. Always closes the log fd."""
        if proc is not None:
            try:
                proc.send_signal(signal.SIGINT)
            except OSError:
                pass
            try:
                proc.wait(timeout=AUDIO_STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                log("audio capture did not stop in %.1fs, killing" % AUDIO_STOP_GRACE_S)
                proc.kill()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            log("audio capture ended: rc=%s" % proc.poll())
        close_fds(log_fd)

    # --- recording ----------------------------------------------------------
    def record(self, run_dir, attempt):
        def out(name):
            return os.path.join(run_dir, retry_name(name, attempt))

        video = out("video.h265")
        audio = out("audio.wav")

        csv_path = out("imu.csv")
        csv_fd = open_output_retry(csv_path, (CSV_HEADER + "\n").encode("ascii"))
        klog_fd = open_output(out("kernel.log"), self.boot_log)
        with self.lock:
            self.csv = csv_fd
            self.csv_path = csv_path
            self.csv_reopen_left = CSV_REOPEN_MAX
            self.klog = klog_fd
            self.row_count = 0
            self.soc_temp = self.read_soc_temp()
        self.set_state(SOC_RECORDING)

        argv = [VICAP_BIN, "-o", video, "-l", str(VICAP_FRAMES), *VICAP_EXTRA_ARGS]
        log("starting capture: %s" % " ".join(argv))

        vicap_log_fd = open_output(out("vicap.log"))
        audio_proc = audio_log_fd = None
        try:
            proc = subprocess.Popen(
                argv,
                stdout=vicap_log_fd if vicap_log_fd is not None else subprocess.DEVNULL,
                stderr=subprocess.STDOUT)
            log("vicap running as pid %d" % proc.pid)
            # Start the mic capture right after vicap so audio and video begin
            # together. Best-effort: it never gates the recording.
            audio_proc, audio_log_fd = self.start_audio(audio)
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
            # Stop audio on every exit (frame-limit, CMD_STOP, shutdown, error)
            # before we release the video log, so the WAV is finalized too.
            self.stop_audio(audio_proc, audio_log_fd)
            close_fds(vicap_log_fd)

        with self.lock:
            self.csv = None
            self.klog = None
        close_fds(csv_fd, klog_fd)
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
            # Stop on an explicit, debounced CMD_STOP -- never on frame absence, so
            # a reception gap or the encoder's slow cold-boot init can't be mistaken
            # for the FC asking us to stop. --record mode ignores commands entirely.
            if not self.force_record and self.command == FRAME_CMD_STOP:
                log("received STOP command, finalizing recording")
                klog("stop: got CMD_STOP, starting graceful stop")
                self.graceful_stop = True
                self.set_state(SOC_STOPPING)
                proc.terminate()   # SIGTERM: vicap flushes and closes the partial
                try:
                    proc.wait(timeout=VICAP_STOP_GRACE_S)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return proc.poll()
            time.sleep(0.2)

    # --- idle / finalize -----------------------------------------------------
    def run_idle(self):
        # CMD_IDLE: this boot is idle, powered up only to service the card (e.g.
        # pull recordings over ssh). Remount /data read-only: a read-only fs is
        # never dirtied, so the stm32 can cut power at any moment -- planned or a
        # surprise yank -- and the card stays clean, with no journal recovery or
        # e2fsck on the next boot. The data is still fully readable over ssh.
        log("IDLE command received, entering idle (not recording)")

        # Our stdout/stderr are the init script's >>/data/startup.log, which
        # holds /data open for writing and makes remount,ro fail EBUSY. Move
        # them to /dev/kmsg (closing the startup.log fd) first, exactly as the
        # finalize path does before it unmounts. log() keeps working, now on
        # the console/kernel log.
        os.sync()
        move_stdio_to_kmsg()

        rc = subprocess.call(["mount", "-o", "remount,ro", DATA_DIR])
        if rc == 0:
            log("/data remounted read-only for idle")
            klog("idle: /data remounted read-only, safe to cut power")
        else:
            log("could not remount /data read-only (rc=%d)" % rc)
            klog("idle: FAILED to remount /data read-only (rc=%d)" % rc)

        # Stay alive so the heartbeat thread keeps emitting SOC_IDLE: that tells
        # the stm32 to stop streaming and park us. Unlike a recording there is
        # nothing to finalize; /data is read-only, so we just wait to be powered
        # off (the stm32 cuts the rails on a later CMD_STOP).
        self.set_state(SOC_IDLE)
        while not self.stop.is_set():
            self.stop.wait(HEARTBEAT_INTERVAL_S)

    def finalize(self, fd, rc):
        """Drive the final state ourselves. A clean finish or graceful stop syncs
        and unmounts /data, then hands off to the stm32 for power-off. A total
        failure keeps reporting SOC_ERROR until the stm32 sends a STOP, then takes
        that same power-off path. force_record stays powered with /data mounted."""
        # A clean frame-limit finish (rc==0) and a graceful stop (got CMD_STOP,
        # partial already saved by vicap on SIGTERM) both mean "recording is
        # safely on disk, cut power". Anything else is a genuine failure.
        done = (rc == 0) or self.graceful_stop

        if self.force_record:
            # Stay powered so an ssh session survives; leave /data mounted.
            self.stop.set()
            if done:
                log("syncing to disk")
                os.sync()
            self.set_state(SOC_STOPPED)
            self.send_state(fd, "SOC_STOPPED")
            os.close(fd)
            return

        if not done:
            # Gave up on a real error. Don't exit -- that kills the heartbeat and
            # leaves the stm32 with no state to act on, so it can never cut our
            # power and the SoC sits there stuck on. Instead keep reporting
            # SOC_ERROR and stay alive until the stm32 sends a STOP (its power-off
            # signal); only then do we fall through to the normal power-off. Leave
            # the reader/heartbeat threads running so we can see that STOP and keep
            # advertising the error, so don't stop them yet.
            klog("finalize: NOT done (rc=%s graceful=%s) -> SOC_ERROR, awaiting STOP"
                 % (rc, self.graceful_stop))
            self.set_state(SOC_ERROR)
            log("all recording attempts failed; reporting SOC_ERROR, waiting for "
                "STOP command before power-off")
            while not self.stop.is_set() and self.command != FRAME_CMD_STOP:
                self.stop.wait(0.2)
            klog("finalize: STOP received (or shutdown), proceeding to power off")

        # Stop the helper threads and drive the final handoff ourselves. Reached
        # by a clean finish, a graceful stop, or a total failure now told to stop.
        self.stop.set()

        # Tell the stm32 we're finalizing, then leave /data cleanly unmounted.
        klog("finalize: done, SOC_STOPPING, about to sync /data")
        self.set_state(SOC_STOPPING)
        self.send_state(fd, "SOC_STOPPING")
        log("syncing and unmounting /data before power-off")
        os.sync()
        klog("finalize: sync() returned, unmounting /data")

        # Our stdout/stderr (incl. the supervisor.log fd) live on /data and would
        # hold it busy, and so would our cwd if it's anywhere under /data. Move
        # both off it first.
        try:
            os.chdir("/")
        except OSError:
            pass
        move_stdio_to_kmsg()

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
        self.set_state(SOC_COMPLETE)
        while True:
            self.send_state(fd)
            time.sleep(HEARTBEAT_INTERVAL_S)

    # --- main --------------------------------------------------------------
    def run(self):
        log("supervisor starting (port=%s)" % SERIAL_PORT)
        # Spare the CPU from a synchronous serial-console printk storm if the
        # camera link degrades mid-run; the ring buffer (and our kernel.log) still
        # get everything. Done first so it protects the whole session.
        quiet_kernel_console()
        fd = open_serial(SERIAL_PORT)

        for handler in (signal.SIGTERM, signal.SIGINT):
            signal.signal(handler, lambda *_: self.stop.set())

        threads = [(self.reader_manager, (fd,)), (self.heartbeat_loop, (fd,))]

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
            log("--record: forcing record, ignoring commands")
        else:
            # Act on the first debounced command; a silent/garbled link times out
            # here and defaults to RECORD (fail toward capturing).
            self.command_seen.wait(COMMAND_WAIT_S)
            if self.command == FRAME_CMD_IDLE:
                self.run_idle()
                return
            log("recording (command=%s)" % CMD_NAMES.get(self.command, self.command))

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
                self.set_state(SOC_ERROR)
                break
            if self.stop.is_set() or self.graceful_stop:
                break
            if rc == 0:
                log("recording finished cleanly")
                break
            self.set_state(SOC_ERROR)
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

        self.finalize(fd, rc)


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