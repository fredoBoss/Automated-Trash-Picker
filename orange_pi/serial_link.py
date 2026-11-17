"""
RoboSort - Serial Link to the Arduino Mega (laptop + Orange Pi One)

The Mega does not care which host is talking to it, so this module is
tested from the laptop over USB long before the Pi exists. Only the port
name differs, and that comes from config.py.

What this side sends (one character + newline, ASCII):
    B\n   biodegradable      -> sorting servo to bio bin
    N\n   non-biodegradable  -> sorting servo to non-bio bin
    X\n   nothing detected
    P\n   heartbeat, once a second - the Mega does not reply, it only
          notes the time so its LCD can show whether the Pi is talking
    T87\n the Pi's CPU temperature in deg C, every TEMP_EVERY_S - for the
          Mega's LCD; no reply
    W30\n the Pi has turned its model off to cool for 30 s - the Mega
          stops the wheels for as long, and counts down on its LCD

The Mega latches B/N and, while driving on its own, stops for one pickup
on a latch younger than 5 s, then clears it. So a command goes out only
when the value changes: one sighting, one pickup. Re-sending B every frame
while the same item stays in view would queue pickup after pickup.

A pickup is bracketed by two lines from the Mega: PICKUP when it stops,
SORTED once the gate is home again (~7.5 s later). Nothing is sent in
between - what the camera sees mid-pickup, the item on the arm or a
flicker, is not a new item. After SORTED, sending resumes against the
last command that went out, so the B that started the pickup is not
repeated: an item still in view after a miss is left behind until the
zone reads X.

F, L and R are the exception to send-on-change. They mean trash is in the
pickup zone but short of the grab spot, and the Mega takes one short step
(forward, or a turn toward it) per line it receives - so each one goes out
every time. vision.py decides when; see its approach notes.

What the Mega sends back (arduino/main/main.ino), one line each:
    RX <line>            a command arrived - printed before it acts
    OK ... / FAIL ...    what it did with it
    INIT ... / AUTO ...  start-up: a 10 s countdown, then it drives itself
    AVOID ...            it turned away from an obstacle
    PICKUP ... / SORTED ...  a pickup started / its item is in the bin
    RoboSort Mega ready  it has just booted
A reader thread prints these with a "mega:" prefix and watches for:
  - The boot banner mid-run: the Mega reset (a supply dip when the motors
    start is the usual cause) and forgot its latch, so the current command
    is sent again on the next frame. A reset also ends any pickup.
  - The Mega entering AUTO ("AUTO - init done" / "OK autonomous"): it
    clears its latch then too, so the current command is sent again.
  - PICKUP / SORTED: commands are held in between. No SORTED within
    PICKUP_TIMEOUT_S means the line was lost, and sending resumes anyway.
  - A command with no "RX" after ACK_WARN_S is reported, not re-sent: a
    duplicate B could start a second pickup. Not checked during a pickup or
    the start-up part checks, when the Mega does not read serial at all; a
    command sent during the checks is timed from their end.

Opening a USB port resets the Mega (the USB-serial chip pulses DTR), which
reruns its start-up: the boot banner, the part checks (~3 s with the LCD,
a missing sonar holds 2 s more), then the 10 s countdown. The Pi's UART
(/dev/ttyS*) does not reset it, and nothing waits for a banner there.
Either way the Mega does not drive off until the first result of this
program's first frame has arrived - B/N/X/F/L/R, not the P heartbeat.

Use DryRunLink to exercise the whole pipeline with no Arduino attached.
"""

import threading
import time

CMD_NAMES = {"B": "bio", "N": "non-bio", "X": "none",
             "F": "step forward", "L": "turn left", "R": "turn right"}
# Approach steps: the Mega makes one short move per line, so these go out
# every time, not only on change.
STEP_COMMANDS = ("F", "L", "R")

MEGA_BANNER = "RoboSort Mega ready"
AUTO_LINES = ("AUTO - init done", "OK autonomous")   # the Mega just entered AUTO
CHECKS_DONE_LINE = "INIT - driving starts"   # last line of setup(), serial is read after it
BOOT_WAIT_S = 6.0   # USB only: bootloader ~2 s + setup (servos homing) ~1 s, with margin


def resets_on_open(port: str) -> bool:
    """Does opening this port reset the Mega? Over USB it does - the
    USB-serial chip pulses DTR. The Pi's own UART (/dev/ttyS*, wired to the
    Mega's Serial1) has no DTR line, so the Mega keeps running."""
    return not port.startswith("/dev/ttyS")
ACK_WARN_S = 3.0
PING = "P"
HEARTBEAT_S = 1.0   # the Mega's LCD calls the Pi lost after 3 s without a line
TEMP_EVERY_S = 2.0  # how often the CPU temperature goes to the Mega's LCD
# The space matters: the Mega's help text has a line starting "PICKUP/SORTED".
PICKUP_LINE = "PICKUP "  # the Mega stopped for an item - hold commands
SORTED_LINE = "SORTED "  # the item is in its bin - send again
PICKUP_TIMEOUT_S = 20.0  # a pickup takes ~7.5 s; no SORTED by then means it was lost


class _BaseLink:
    """Send-on-change bookkeeping, shared by the real and dry-run links."""

    def __init__(self):
        self.last_command: str | None = None
        self.sent_count = 0
        self.resets = 0
        # Set by the reader when the Mega reboots. A flag rather than clearing
        # last_command: a send() already in flight would overwrite that.
        self._resync = False
        # Set by the reader from PICKUP to SORTED: when to stop waiting for
        # SORTED. 0 when no pickup is running.
        self._busy_until = 0.0

    @property
    def busy(self) -> bool:
        """The Mega is mid-pickup, between PICKUP and SORTED."""
        return time.monotonic() < self._busy_until

    def send(self, command: str, force: bool = False) -> bool:
        """Returns True if the command actually went out on the wire.
        Held while the Mega is mid-pickup, unless forced."""
        if self.busy and not force:
            return False
        if self._resync:
            self._resync = False
            force = True
        if command == self.last_command and not force and command not in STEP_COMMANDS:
            return False
        self._write(command)
        self.last_command = command
        self.sent_count += 1
        return True

    def _write(self, command: str) -> None:
        raise NotImplementedError

    def cooling(self, seconds: float) -> None:
        """Tell the Mega the model is off for `seconds` to cool: it stops
        the wheels for as long. Not a command - no RX is awaited, and
        last_command is untouched."""
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SerialLink(_BaseLink):
    def __init__(self, port: str, baud: int, retries: int = 5, echo: bool = True,
                 temp_source=None):
        super().__init__()
        import serial          # late import so DryRunLink works without pyserial

        self.echo = echo
        self._temp_source = temp_source    # () -> deg C or None; None = send no T
        self._last_temp_tx = 0.0
        self._lock = threading.Lock()
        self._wlock = threading.Lock()
        self._last_tx = 0.0
        self._live = False                 # heartbeats start once the boot wait is over
        self._pending: tuple[str, float] | None = None   # (command, sent at)
        self._warned = False
        self._starting = False             # boot banner seen, part checks not over yet
        self._booted = threading.Event()
        self._stop = threading.Event()

        last_error = None
        for attempt in range(1, retries + 1):
            try:
                self.ser = serial.Serial(port, baud, timeout=0.2)
                break
            except serial.SerialException as e:
                last_error = e
                print(f"  serial attempt {attempt}/{retries} failed: {e}")
                time.sleep(2)
        else:
            raise RuntimeError(f"Cannot open serial port {port}: {last_error}")

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        if not resets_on_open(port):
            # The Pi's UART: opening it does not reset the Mega, so there is no
            # banner to wait for. Waiting BOOT_WAIT_S here only delayed the
            # first frame by 6 s on every start - and the Mega now holds its
            # wheels until that frame's result arrives (main.ino visionReady).
            print(f"Serial open: {port} @ {baud} (UART - no Mega reset)")
        else:
            # USB: opening the port reset the Mega, and anything written while
            # its bootloader runs is lost - wait for the banner. No banner means
            # it is not answering; carry on either way.
            print(f"Serial open: {port} @ {baud} - waiting for the Mega to boot")
            if self._booted.wait(BOOT_WAIT_S):
                print("Mega is up - part checks, then it drives once vision is running")
            else:
                print(f"  no boot banner within {BOOT_WAIT_S:.0f} s - carrying on")
        self._live = True

    def _write(self, command: str) -> None:
        with self._lock:
            self._pending = (command, time.monotonic())
            self._warned = False
        self._tx(command)

    def _tx(self, command: str) -> None:
        # Two threads write: send() from the main loop, heartbeats from the
        # reader. The lock keeps their bytes from interleaving on the wire.
        with self._wlock:
            self.ser.write(f"{command}\n".encode())
            self.ser.flush()
            self._last_tx = time.monotonic()

    def _heartbeat(self) -> None:
        """P once a second, so the Mega's LCD can tell a live Pi from a
        silent one - B/N/X only go out on change, and X can hold for
        minutes. Any command resets the clock, so P only fills the gaps.
        Nothing is sent before the boot wait is over: bytes that reach the
        bootloader are lost at best."""
        if not self._live:
            return
        try:
            now = time.monotonic()
            if self._temp_source and now - self._last_temp_tx >= TEMP_EVERY_S:
                self._last_temp_tx = now
                temp = self._temp_source()
                if temp is not None:
                    self._tx(f"T{temp:.0f}")   # counts as a heartbeat too
            if time.monotonic() - self._last_tx >= HEARTBEAT_S:
                self._tx(PING)
        except Exception:
            pass                             # a dead port is reported by the read side

    def cooling(self, seconds: float) -> None:
        try:
            self._tx(f"W{seconds:.0f}")
        except Exception:
            pass                             # a dead port is reported by the read side

    # ------------------------------------------------------------ reader
    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            self._heartbeat()
            try:
                chunk = self.ser.readline()
            except Exception as e:           # unplugged, or closed under us
                if not self._stop.is_set():
                    print(f"  mega: serial read failed ({e}) - link lost")
                return
            if not chunk:
                self._check_ack()
                self._check_pickup()
                continue
            buf += chunk
            if not buf.endswith(b"\n"):
                continue                     # timed out mid-line, keep it
            line = buf.decode("ascii", errors="replace").strip()
            buf = b""
            if line:
                self._on_line(line)

    def _on_line(self, line: str) -> None:
        if self.echo:
            print(f"  mega: {line}")

        if line.startswith(MEGA_BANNER):
            if self._booted.is_set():
                self.resets += 1
                print(f"  mega: RESET #{self.resets} - it forgot its latch, "
                      f"{self.last_command!r} goes out again on the next frame")
                with self._lock:
                    self._pending = None
                self._resync = True          # makes the next send() go out
            self._busy_until = 0.0           # a reset ends any pickup
            self._starting = True
            self._booted.set()
            return

        # The part checks are over and the Mega reads serial from here on, so
        # a command sent during them only starts waiting for its RX now.
        if line.startswith(CHECKS_DONE_LINE):
            with self._lock:
                self._starting = False
                if self._pending:
                    self._pending = (self._pending[0], time.monotonic())
            return

        # The Mega reads no serial during a pickup, and anything sent then
        # is dropped once it is over - hold everything until SORTED.
        if line.startswith(PICKUP_LINE):
            self._busy_until = time.monotonic() + PICKUP_TIMEOUT_S
            print("  pi: holding commands until the Mega reports SORTED")
            return
        if line.startswith(SORTED_LINE):
            self._busy_until = 0.0
            return

        # Entering AUTO (the countdown ran out, or 'A') drops the Mega's
        # latch on purpose, so a B/N from a bench test cannot fire a pickup.
        # Trash the camera sees right now is not stale, though: send the
        # current command again, or an item already in the pickup zone
        # when driving starts would never be collected.
        if line.startswith(AUTO_LINES):
            self._resync = True
            return

        if line.startswith("RX "):
            got = line[3:].strip()
            with self._lock:
                self._starting = False       # it is reading serial, whatever was missed
                if self._pending and got == self._pending[0].replace(" ", "").upper():
                    self._pending = None
                    self._warned = False

    def _check_ack(self) -> None:
        with self._lock:
            p = self._pending
            if (p is None or self._warned or self.busy or self._starting
                    or time.monotonic() - p[1] < ACK_WARN_S):
                return
            self._warned = True
        print(f"  mega: no 'RX {p[0]}' after {ACK_WARN_S:.0f} s - the Mega is "
              f"busy (pickup or turn) or the link dropped; not re-sending")

    def wait_rx(self, timeout: float) -> bool:
        """Block until the Mega has echoed the last command sent (its RX
        line) or timeout runs out. Right after a reset the Mega spends ~3 s
        on its part checks before it reads serial at all."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lock:
                if self._pending is None:
                    return True
            time.sleep(0.05)
        return False

    def _check_pickup(self) -> None:
        if self._busy_until and not self.busy:
            self._busy_until = 0.0
            print(f"  mega: no SORTED within {PICKUP_TIMEOUT_S:.0f} s - "
                  f"the line was lost; sending again")

    def close(self) -> None:
        try:
            self.send("X", force=True)       # leave the gate in a known state
            time.sleep(0.3)                  # let the reader print the answer
        except Exception:
            pass
        self._live = False
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        self._reader.join(timeout=1)


class DryRunLink(_BaseLink):
    """No Arduino attached - prints what would have been sent. This is the
    default on the laptop until the Mega is wired up."""

    def __init__(self):
        super().__init__()
        print("Serial: DRY RUN (nothing is sent to an Arduino)")

    def _write(self, command: str) -> None:
        print(f"  [dry-run] would send {command!r} ({CMD_NAMES.get(command, '?')})")

    def cooling(self, seconds: float) -> None:
        print(f"  [dry-run] would send 'W{seconds:.0f}' (cooling - wheels stop)")


def open_link(port: str, baud: int, retries: int = 5, dry_run: bool = False,
              temp_source=None):
    if dry_run:
        return DryRunLink()
    return SerialLink(port, baud, retries, temp_source=temp_source)
