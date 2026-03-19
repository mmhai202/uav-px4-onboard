# UAV PX4 Onboard Mission

This repository provides a custom PX4 onboard module (`my_mission`) as a clean starting point for developing and testing autonomous flight algorithms directly on Pixhawk.
- Build a shared repo where everyone can develop onboard UAV algorithms.
- Run mission logic inside PX4 firmware (no external offboard control loop required).
- Keep deployment simple for both SITL and real hardware.

## Setup
1. Install PX4 firmware (v1.15.1) and Gazebo-Classic 11
2. Add `my_mission` to `PX4-Autopilot/src/modules/`.
3. Add model
- Set model param: Place file `10050_gazebo-classic_my_vehicle` in the following path, then add it to `px4_add_romfs_files()` inside `CMakeLists.txt`  
*home/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes*
- Add model: 
    - Add folder `my_vehicle` in the following path  
    *home/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models*
    - Then open file `sitl_targets_gazebo-classic.cmake` in the following path, and add `my_vehicle` to `set(models)`  
    *home/PX4-Autopilot/src/modules/simulation/simulator_mavlink*
- Set model to the origin in simulation: Open file `sitl_run.sh` in the following path, then search for "spawn-file" and change (x,y,z) to `-x 0.0 -y 0.0 -z 0.83`  
*home/PX4-Autopilot/Tools/simulation/gazebo-classic*
4. Recommended params (included in `10050_gazebo-classic_my_vehicle`, if use TFMINI to estimate Z)
```txt
SENS_TFMINI_CFG = 103 # TELEM 3
EKF2_HGT_REF = 2
EKF2_RNG_CTRL = 2
EKF2_RNG_A_HMAX = 10.000
EKF2_RNG_A_VMAX = 2.0
```
5. Add to `PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.mc_apps`:
```txt
my_mission start
```

6. Config module
```txt
CONFIG_MODULES_MY_MISSION=y

# Simulation Setup
# Add to `PX4-Autopilot/boards/px4/sitl/default.px4board`

# Real flight setup (FMU-v6x)
# Add to `PX4-Autopilot/boards/px4/fmu-v6x/default.px4board`
```

7. Build and flash (Pixhawk)
```bash
make -j px4_fmu-v6x_default
make px4_fmu-v6x_default upload
```

## Build and Run
```bash
# build
cd PX4-Autopilot
make px4_sitl_default gazebo-classic_my_vehicle
```
Enters **MISSON MODE**, `my_mission` runs automatically.
If switched to **HOLD**, the mission is aborted and the vehicle hovers.
```bash
# run
commander mode auto:mission # mission mode

# stop
commander mode auto:loiter # hold mode
```
![Demo Animation](images/demo.gif)