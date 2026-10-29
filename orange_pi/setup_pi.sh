#!/bin/sh
# RoboSort - one-time runtime setup on the Orange Pi One (Armbian, Debian 13).
# Run on the Pi from anywhere:
#     sh ~/robosort/orange_pi/setup_pi.sh
# Safe to re-run. Afterwards start things with ~/robosort/.venv/bin/python.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/../.venv"
SUDO=""
[ "$(id -u)" -eq 0 ] || SUDO="sudo"

# Debian's armhf builds - PyPI has none of these for this CPU and Python.
$SUDO apt-get update
$SUDO apt-get install -y python3-venv python3-pip python3-numpy python3-opencv python3-serial

# Camera and Arduino access for a normal user (root has both already).
if [ "$(id -u)" -ne 0 ]; then
    $SUDO usermod -aG video,dialout "$(id -un)"
fi

# --system-site-packages lets the venv see the apt numpy/OpenCV/pyserial.
python3 -m venv --system-site-packages "$VENV"
# --no-deps: ncnn lists opencv-python as a dependency, which has no armv7l
# wheel - pip would try to compile it from source and fail (no compiler).
# `import ncnn` needs none of its deps; only its model-zoo helpers do.
"$VENV/bin/pip" install --no-deps -r "$HERE/requirements-pi.txt"

"$VENV/bin/python" -c "import ncnn, numpy, cv2, serial; print('RoboSort Pi runtime OK: ncnn', ncnn.__version__, '| numpy', numpy.__version__, '| opencv', cv2.__version__, '| pyserial', serial.__version__)"
