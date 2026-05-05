#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PX4_DIR=${1:-"${REPO_ROOT}/../PX4-Autopilot"}

cp -rf "${REPO_ROOT}/my_mission" \
    "${PX4_DIR}/src/modules/"
cp -f "${REPO_ROOT}/setup/my_world/my_world.world" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/my_world.world"
cp -rf "${REPO_ROOT}/setup/my_vehicle" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/"
cp -f "${REPO_ROOT}/setup/10050_gazebo-classic_my_vehicle" \
    "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10050_gazebo-classic_my_vehicle"
cp -f "${REPO_ROOT}/setup/CMakeLists.txt" \
    "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
cp -f "${REPO_ROOT}/setup/sitl_targets_gazebo-classic.cmake" \
    "${PX4_DIR}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
cp -f "${REPO_ROOT}/setup/sitl_run.sh" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_run.sh"
cp -f "${REPO_ROOT}/setup/rc.mc_apps" \
    "${PX4_DIR}/ROMFS/px4fmu_common/init.d/rc.mc_apps"
cp -f "${REPO_ROOT}/setup/board_sitl/default.px4board" \
    "${PX4_DIR}/boards/px4/sitl/default.px4board"
cp -f "${REPO_ROOT}/setup/board_fmu-v6x/default.px4board" \
    "${PX4_DIR}/boards/px4/fmu-v6x/default.px4board"

echo "PX4 setup complete: ${PX4_DIR}"
