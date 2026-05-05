# UAV PX4 Onboard Mission

This repository provides a custom PX4 onboard module (`my_mission`) as a clean starting point for developing and testing autonomous flight algorithms directly on Pixhawk.
- Build a shared repo where everyone can develop onboard UAV algorithms.
- Run mission logic inside PX4 firmware (no external offboard control loop required).
- Keep deployment simple for both SITL and real hardware.

## Setup
1. Setup PX4 firmware (`v1.15.1`) and Gazebo-Classic 11:
```bash
chmod +x scripts/setup_firmware.sh
./scripts/setup_firmware.sh
```
The script uses the parent folder of this repo and creates `PX4-Autopilot` there automatically.
This script is intended for Ubuntu, requires `sudo`, and installs PX4 `v1.15.1` with Gazebo-Classic 11.
It uses a recursive PX4 clone with `--depth 1` for a simpler and more compatible setup.
If `pip` is already installed, the script downgrades it to a PX4 `v1.15.1` compatible version before running `ubuntu.sh`.

2. Copy this repo's module and PX4 config files:
```bash
chmod +x scripts/setup_px4.sh
./scripts/setup_px4.sh
```
The script automatically targets `../PX4-Autopilot`.
This script overwrites the matching PX4 config files and is intended for PX4 `v1.15.1`.
The target `PX4-Autopilot` tree must already exist with the normal PX4 folder structure.

The script copies:
- `my_mission` to `PX4-Autopilot/src/modules/`
- `my_vehicle` model and `my_world.world`
- airframe `10050_gazebo-classic_my_vehicle`
- `CMakeLists.txt`, `sitl_targets_gazebo-classic.cmake`, `sitl_run.sh`
- `rc.mc_apps`
- `boards/px4/sitl/default.px4board`
- `boards/px4/fmu-v6x/default.px4board`

3. Recommended params:
- SITL already includes:
```txt
EKF2_HGT_REF = 2
EKF2_RNG_CTRL = 2
EKF2_RNG_A_HMAX = 10.000
EKF2_RNG_A_VMAX = 2.0
```
- Real flight with TFMINI for Z estimation also needs:
```txt
SENS_TFMINI_CFG = 103 # TELEM 3
```
4. Build and flash (Pixhawk)
```bash
make -j px4_fmu-v6x_default
make px4_fmu-v6x_default upload
```

## Build and Run
```bash
# build
cd ../PX4-Autopilot
make -j px4_sitl_default gazebo-classic_my_vehicle
```
Enters **MISSION MODE**, `my_mission` runs automatically.
If switched to **HOLD**, the mission is aborted and the vehicle hovers.
```bash
# run
commander mode auto:mission # mission mode
# stop
commander mode auto:loiter # hold mode
```
![Demo Animation](images/demo.gif)
