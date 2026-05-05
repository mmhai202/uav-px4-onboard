#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEV_ROOT=${1:-"${REPO_ROOT}/.."}

cd "${DEV_ROOT}"

if [ ! -d PX4-Autopilot ]; then
    git clone -b v1.15.1 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
fi

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt remove -y gz-harmonic
sudo apt install -y aptitude
sudo aptitude install -y gazebo libgazebo11 libgazebo-dev

echo "Firmware setup complete!"
