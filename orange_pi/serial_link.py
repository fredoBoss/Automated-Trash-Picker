"""
RoboSort - Serial Link to the Arduino Mega (laptop + Orange Pi One)

The Mega does not care which host is talking to it, so this module is
tested from the laptop over USB long before the Pi exists. Only the port
name differs, and that comes from config.py.

What this side sends (one character + newline, ASCII):
    B\n   biodegradable      -> sorting servo to bio bin
    N\n   non-biodegradable  -> sorting servo to non-bio bin
    X\n   nothing detected

The Mega latches B/N and, while driving on its own, stops for one pickup
on a latch younger than 5 s, then clears it. So a command goes out only
when the value changes: one sighting, one pickup. Re-sending B every frame
while the same item stays in view would queue pickup after pickup.

What the Mega sends back (arduino/main/main.ino), one line each:
    RX <line>            a command arrived - printed before it acts
    OK ... / FAIL ...    what it did with it
    INIT ... / AUTO ...  start-up: a 10 s countdown, then it drives itself
    AVOID ...            it turned away from an obstacle
    RoboSort Mega ready  it has just booted
A reader thread prints these with a "mega:" prefix and watches for:
  - The boot banner mid-run: the Mega reset (a supply dip when the motors
    start is the usual cause) and forgot its latch, so the current command
    is sent again on the next frame.
  - The Mega entering AUTO ("AUTO - init done" / "OK autonomous"): it
    clears its latch then too, so the current command is sent again.
  - A command with no "RX" after ACK_WARN_S is reported, not re-sent: the
    Mega goes quiet for ~10 s during a pickup, and a duplicate B would
    start a second one.

Opening the port resets the Mega (the USB-serial chip pulses DTR), which
restarts its 10 s start-up countdown - the rover drives off 10 s after
this program connects.

Use DryRunLink to exercise the whole pipeline with no Arduino attached.
"""

import threading
import time

CMD_NAMES = {"B": "bio", "N": "non-bio", "X": "none"}

MEGA_BANNER = "RoboSort Mega ready"
AUTO_LINES = ("AUTO - init done", "OK autonomous")   # the Mega just entered AUTO
BOOT_WAIT_S = 6.0   # bootloader ~2 s + setup (servos homing) ~1 s, with margin
ACK_WARN_S = 3.0


class _BaseLink:
    """Send-on-change bookkeeping, shared by the real and dry-run links."""

    def __init__(self):
        self.last_command: str | None = None
        self.sent_count = 0
        self.resets = 0
        # Set by the reader when the Mega reboots. A flag rather than clearing
        # last_command: a send() already in flight would overwrite that.
        self._resync = False

    def send(self, command: str, force: bool = False) -> bool:
        """Returns True if the command actually went out on the wire."""
        if self._resync:
            self._resync = False
            force = True
        if command == self.last_command and not force:
            return False
        self._write(command)
        self.last_command = command
        self.sent_count += 1
        return True

    def _write(self, command: str) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SerialLink(_BaseLink):
    def __init__(self, port: str, baud: int, retries: int = 5, echo: bool = True):
        super().__init__()
        import serial          # late import so DryRunLink works without pyserial

        self.echo = echo
        self._lock = threading.Lock()
        self._pending: tuple[str, float] | None = None   # (command, sent at)
        self._warned = False
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

        print(f"Serial open: {port} @ {baud} - waiting for the Mega to boot")
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        # Anything written while the bootloader runs is lost, so wait for the
        # banner rather than a fixed sleep. No banner means the port did not
        # reset it (or it is not answering) - carry on either way.
        if self._booted.wait(BOOT_WAIT_S):
            print("Mega is up - its 10 s start-up countdown is running")
        else:
            print(f"  no boot banner within {BOOT_WAIT_S:.0f} s - carrying on")

    def _write(self, command: str) -> None:
        with self._lock:
            self._pending = (command, time.monotonic())
            self._warned = False
        self.ser.write(f"{command}\n".encode())
        self.ser.flush()

    # ------------------------------------------------------------ reader
    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.readline()
            except Exception as e:           # unplugged, or closed under us
                if not self._stop.is_set():
                    print(f"  mega: serial read failed ({e}) - link lost")
                return
            if not chunk:
                self._check_ack()
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
            self._booted.set()
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
                if self._pending and got == self._pending[0].replace(" ", "").upper():
                    self._pending = None
                    self._warned = False

    def _check_ack(self) -> None:
        with self._lock:
            p = self._pending
            if p is None or self._warned or time.monotonic() - p[1] < ACK_WARN_S:
                return
            self._warned = True
        print(f"  mega: no 'RX {p[0]}' after {ACK_WARN_S:.0f} s - the Mega is "
              f"busy (pickup or turn) or the link dropped; not re-sending")

    def close(self) -> None:
        try:
            self.send("X", force=True)       # leave the gate in a known state
            time.sleep(0.3)                  # let the reader print the answer
        except Exception:
            pass
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


def open_link(port: str, baud: int, retries: int = 5, dry_run: bool = False):
    if dry_run:
        return DryRunLink()
    return SerialLink(port, baud, retries)
