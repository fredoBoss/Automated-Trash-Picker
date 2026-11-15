#!/bin/sh
# RoboSort - start vision.py every time the Pi boots, as a systemd service.
# Run once on the Pi:
#     sh ~/robosort/orange_pi/install_autostart.sh
# Safe to re-run: it rewrites the service and restarts it.
#
# While the service runs it holds the camera and the Mega's serial port, so
# test_cv.py, mega.py or a second vision.py cannot open them. Stop it first:
#     systemctl stop robosort-vision        (until the next boot)
#     systemctl start robosort-vision
#     journalctl -u robosort-vision -f      (what it prints)
#     systemctl disable --now robosort-vision   (no autostart any more)
set -e
SUDO=""
[ "$(id -u)" -eq 0 ] || SUDO="sudo"

HERE=$(cd "$(dirname "$0")" && pwd)
PY="$HERE/../.venv/bin/python"
[ -x "$PY" ] || PY=$(command -v python3)
UNIT=/etc/systemd/system/robosort-vision.service

$SUDO tee "$UNIT" >/dev/null <<EOF
[Unit]
Description=RoboSort vision pipeline - camera, model, and the B/N/X/P link to the Mega
# The camera and the UART are local; nothing here needs the network.
After=multi-user.target

[Service]
Type=simple
WorkingDirectory=$HERE
# ROBOSORT_SERIAL_PORT and any other ROBOSORT_* settings.
EnvironmentFile=-/etc/environment
Environment=PYTHONUNBUFFERED=1
ExecStart=$PY vision.py --no-display
# A missing camera or a crash: try again, without hammering the CPU.
Restart=always
RestartSec=10
# Leave CPU time for SSH and the system when the model is busy.
Nice=5

[Install]
WantedBy=multi-user.target
EOF

$SUDO systemctl daemon-reload
$SUDO systemctl enable robosort-vision >/dev/null 2>&1
$SUDO systemctl restart robosort-vision
sleep 2

echo "Installed: $UNIT"
echo "  runs:   $PY vision.py --no-display  (in $HERE)"
echo "  state:  $(systemctl is-active robosort-vision), starts at every boot"
echo "  serial: $(grep '^ROBOSORT_SERIAL_PORT=' /etc/environment 2>/dev/null | cut -d= -f2 || echo 'auto-detect (USB only)')"
echo "Logs:     journalctl -u robosort-vision -f"
echo "Stop it before running test_cv.py or mega.py:  systemctl stop robosort-vision"
