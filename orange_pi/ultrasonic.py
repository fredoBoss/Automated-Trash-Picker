"""
RoboSort - Ultrasonic Range Test (Orange Pi 1)
Bench test for a single HC-SR04. Prints a distance reading once per
second until Ctrl+C.

NOTE: in the shipped design all 4 ultrasonics live on the Arduino Mega,
because obstacle avoidance has to halt the rover without waiting on the
Pi. Linux is not real-time - a scheduler stall here shows up as a wrong
distance, not a late one. This script is for bench-checking a sensor,
not for the avoidance loop.

WIRING (Orange Pi One 40-pin header - same header as Orange Pi PC)
    VCC   -> Arduino Mega 5V   sensor needs 5V, 3.3V will not fire it
    GND   -> Arduino Mega GND
    GND   -> Pi pin 6 (0v)     *** REQUIRED - see common ground below ***
    TRIG  -> pin 12  "GPIO.1"  = PD14, 3.3V drive is fine for a sensor input
    ECHO  -> pin 7   "GPIO.7"  = PA6, *** VIA DIVIDER - SEE BELOW ***

COMMON GROUND
Powering the sensor from the Mega is fine, but ECHO is then referenced to
the MEGA's ground while the Pi reads it against the PI's ground. With no
wire between the two grounds that voltage is undefined: readings come out
as garbage or permanent "no echo". Run a jumper from any Mega GND to any
Pi 0v pin (6, 9, 14, 20, 25, 30, 34, 39).

ECHO idles at 5V logic. Orange Pi GPIO is 3.3V and NOT 5V tolerant, so
ECHO must go through a divider or you risk the pin / the SoC:

    ECHO --+-- 1k --+-- 2k -- GND
           |        |
       (sensor)   pin 7         5.0V * 2k/(1k+2k) = 3.33V

Requires libgpiod v2 (Debian 13 / kernel 6.x - sysfs GPIO is gone):
    sudo apt install -y python3-libgpiod
    python3 ultrasonic.py
"""

import statistics
import sys
import time

import gpiod
from gpiod.line import Direction, Value

# --- Configuration ---
# libgpiod uses the SoC line number, not the physical pin and not the WPi
# number. H3 banks are 32 lines each: PA=0, PB=32, PC=64, PD=96, PE=128...
CHIP        = "/dev/gpiochip0"
TRIG_LINE   = 110    # PD14 = 96 + 14 -> physical pin 12, table name "GPIO.1"
ECHO_LINE   = 6      # PA6  =  0 +  6 -> physical pin 7,  table name "GPIO.7"

SAMPLES     = 5      # readings per printed value, median-filtered
INTERVAL    = 1.0    # seconds between printed values
SETTLE      = 0.06   # HC-SR04 needs >60ms between pings or echoes overlap

TRIG_PULSE  = 10e-6  # 10us trigger pulse, per datasheet
ECHO_TIMEOUT = 0.03  # 30ms ~= 5m, beyond the sensor's 4m range

# Round trip: distance = (t * speed_of_sound) / 2, 343 m/s at 20C
CM_PER_SECOND = 34300.0
MIN_CM, MAX_CM = 2.0, 400.0    # HC-SR04 rated range


def measure_once(request) -> float | None:
    """Fire one ping. Returns distance in cm, or None on timeout/out-of-range."""
    # 10us high pulse on TRIG starts a measurement
    request.set_value(TRIG_LINE, Value.ACTIVE)
    time.sleep(TRIG_PULSE)
    request.set_value(TRIG_LINE, Value.INACTIVE)

    # Wait for ECHO to go high. If it never does, the sensor is not
    # responding - unpowered, miswired, or dead.
    deadline = time.perf_counter() + ECHO_TIMEOUT
    while request.get_value(ECHO_LINE) == Value.INACTIVE:
        if time.perf_counter() > deadline:
            return None

    # ECHO stays high for a duration proportional to distance.
    start = time.perf_counter()
    deadline = start + ECHO_TIMEOUT
    while request.get_value(ECHO_LINE) == Value.ACTIVE:
        if time.perf_counter() > deadline:
            return None      # stuck high - nothing came back, target too far
    elapsed = time.perf_counter() - start

    distance = (elapsed * CM_PER_SECOND) / 2.0
    if not (MIN_CM <= distance <= MAX_CM):
        return None
    return distance


def measure(request, samples: int = SAMPLES) -> float | None:
    """Median of several pings. Polling GPIO from Python is jittery, and a
    single stalled read reads as a falsely large distance - the median
    throws those out."""
    readings = []
    for _ in range(samples):
        d = measure_once(request)
        if d is not None:
            readings.append(d)
        time.sleep(SETTLE)

    if not readings:
        return None
    return statistics.median(readings)


def main():
    try:
        request = gpiod.request_lines(
            CHIP,
            consumer="robosort-ultrasonic",
            config={
                TRIG_LINE: gpiod.LineSettings(
                    direction=Direction.OUTPUT, output_value=Value.INACTIVE
                ),
                ECHO_LINE: gpiod.LineSettings(direction=Direction.INPUT),
            },
        )
    except (OSError, PermissionError) as e:
        sys.exit(f"Cannot open {CHIP}: {e}\nTry: sudo python3 ultrasonic.py")

    print(f"HC-SR04 on {CHIP}  TRIG=line{TRIG_LINE} (pin 12)  ECHO=line{ECHO_LINE} (pin 7)")
    print("Reading... Ctrl+C to stop.\n")

    misses = 0
    try:
        with request:
            while True:
                distance = measure(request)

                if distance is None:
                    misses += 1
                    print(f"  --.- cm   no echo ({misses} in a row)")
                    if misses == 5:
                        print("    5 misses - check: Mega GND jumpered to a Pi 0v pin,")
                        print("    5V on VCC, and the ECHO divider on pin 7")
                else:
                    misses = 0
                    bar = "#" * min(int(distance / 5), 40)
                    print(f"  {distance:5.1f} cm   {bar}")

                time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
