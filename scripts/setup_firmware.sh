#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEV_ROOT=${1:-"${REPO_ROOT}/.."}

cd "${DEV_ROOT}"

if [ ! -d PX4-Autopilot ]; then
    git clone -b v1.15.1 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
fi

# PX4 v1.15.1 still ships legacy Python requirement specifiers such as
# "matplotlib>=3.0.*", which newer pip releases reject. Only downgrade pip
# when it already exists; on a fresh Ubuntu machine ubuntu.sh will install it.
if python3 -m pip --version >/dev/null 2>&1; then
    python3 -m pip install --user "pip<24"
fi

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt remove -y gz-harmonic
sudo apt install -y aptitude
sudo aptitude install -y gazebo libgazebo11 libgazebo-dev

echo "Firmware setup complete!"
