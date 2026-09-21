#!/usr/bin/env bash
# Open the AVIAN REV-C corridor in Gazebo Harmonic.
#
#   ./view.sh            launch the GUI
#   ./view.sh --check    parse the world and exit  (gz sdf -k)
#   ./view.sh --headless run 100 iterations headless and exit
#
# The ROS sourcing is not optional. gz on this machine is the ROS-vendored
# build at /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz, and without the ROS
# environment it reports:
#     I cannot find any available 'gz' command
#     * Did you install any Gazebo library?
# which reads like Gazebo is missing when it is installed and working.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${HERE}/worlds/avian_sic.sdf"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    # set +u around the sourcing: ROS's setup.bash reads
    # AMENT_TRACE_SETUP_FILES without a default, which is fatal under
    # `set -u` and aborts before gz is ever reached.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
fi

# Models are referenced as model://avian_bridge/... and resolved from here.
# Nothing in the SDF carries an absolute path, so the world is portable --
# V50 fails the gate on any /home in any SDF.
export GZ_SIM_RESOURCE_PATH="${HERE}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
# SDF_PATH as well, and it is not redundant: `gz sim` resolves model:// via
# GZ_SIM_RESOURCE_PATH, but the standalone `gz sdf -k` validator uses
# sdformat's own findFile, which reads SDF_PATH. Without it the validator
# reports "Unable to find uri[model://avian_bridge]" and exits 255 on a
# world that is perfectly good.
export SDF_PATH="${HERE}/models${SDF_PATH:+:${SDF_PATH}}"

if [ ! -f "${WORLD}" ]; then
    echo "No world at ${WORLD} -- run export_gazebo.py first:" >&2
    echo "  cd ${HERE}/.. && blender -b --python run_blender.py -- \\" >&2
    echo "      source/export_gazebo.py" >&2
    exit 1
fi

case "${1:-}" in
    --check)
        echo "gz sdf -k ${WORLD}"
        exec gz sdf -k "${WORLD}"
        ;;
    --headless)
        echo "gz sim -s -r --iterations 100 ${WORLD}"
        exec gz sim -s -r --iterations 100 "${WORLD}"
        ;;
    *)
        echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
        echo "gz sim -v 4 ${WORLD}"
        echo
        echo "The world loads PAUSED. Press Play, bottom left, to start"
        echo "physics. Scroll to zoom, middle-drag to orbit."
        exec gz sim -v 4 "${WORLD}"
        ;;
esac
