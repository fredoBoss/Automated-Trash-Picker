#!/bin/sh
# RoboSort - one-time setup of the Pi end of the Mega link: UART3 on header
# pins 8 (TX3) and 10 (RX3), which shows up as /dev/ttyS3.
# Run on the Pi:
#     sh ~/robosort/orange_pi/setup_uart.sh
# Safe to re-run. Reboot afterwards when it says so.
set -e

SUDO=""
[ "$(id -u)" -eq 0 ] || SUDO="sudo"
BOOT_ENV=/boot/armbianEnv.txt
PORT=/dev/ttyS3

# Append a line to a root-owned file, starting a new line first if the file
# does not end in one - otherwise the two lines would run together.
append() {
    if [ -s "$2" ] && [ -n "$(tail -c1 "$2")" ]; then
        echo | $SUDO tee -a "$2" >/dev/null
    fi
    echo "$1" | $SUDO tee -a "$2" >/dev/null
}

# 1. UART3 itself. /dev/ttyS0-7 all exist whether or not their UART is on,
#    so ask the device tree instead. Armbian 26.5's Orange Pi One image turns
#    UART3 (serial@1c28c00, pins 8/10) on by itself; older images need the
#    uart3 overlay, which takes effect after a reboot.
uart3_on() {
    [ "$(tr -d '\0' < /proc/device-tree/soc/serial@1c28c00/status 2>/dev/null)" = okay ]
}
if uart3_on; then
    echo "UART3:  already on (serial@1c28c00, pins 8/10) - boot config left alone"
elif grep -Eq '^overlays=(.* )?uart3( |$)' "$BOOT_ENV"; then
    echo "UART3:  overlay already in $BOOT_ENV - reboot to turn it on"
else
    prefix=$(sed -n 's/^overlay_prefix=//p' "$BOOT_ENV")
    if [ ! -f "/boot/dtb/overlay/${prefix:-sun8i-h3}-uart3.dtbo" ]; then
        echo "Warning: /boot/dtb/overlay/${prefix:-sun8i-h3}-uart3.dtbo not found -" \
             "check that this is Armbian on the Orange Pi One." >&2
    fi
    $SUDO cp "$BOOT_ENV" "$BOOT_ENV.bak"
    if grep -q '^overlays=' "$BOOT_ENV"; then
        $SUDO sed -i -e 's/^overlays=\(.*\)$/overlays=\1 uart3/' \
                     -e 's/^overlays= /overlays=/' "$BOOT_ENV"
    else
        append "overlays=uart3" "$BOOT_ENV"
    fi
    echo "UART3:  overlay added to $BOOT_ENV (previous copy: $BOOT_ENV.bak)"
    grep '^overlays=' "$BOOT_ENV" | sed 's/^/        /'
fi

# 2. The port name. config.py only auto-detects USB serial ports, so name
#    the UART for every login, including commands run over ssh.
if grep -q '^ROBOSORT_SERIAL_PORT=' /etc/environment 2>/dev/null; then
    $SUDO sed -i "s|^ROBOSORT_SERIAL_PORT=.*|ROBOSORT_SERIAL_PORT=$PORT|" /etc/environment
else
    append "ROBOSORT_SERIAL_PORT=$PORT" /etc/environment
fi
echo "Port:   ROBOSORT_SERIAL_PORT=$PORT in /etc/environment (from the next login)"

if uart3_on; then
    echo "Done:   UART3 is on."
else
    echo "Next:   sudo reboot - UART3 turns on during it."
fi
echo "Test:   log in again, then"
echo "        cd ~/robosort/orange_pi && ../.venv/bin/python mega.py ?"
