# wcr_stack

ROS 2 Humble stack for a wall-climbing robot with a 4WIS4WID (four-wheel independent steering, four-wheel independent drive) kinematic structure: control, odometry, hardware drivers and bringup.

## Requirements

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Build tools:
  ```bash
  sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions
  sudo rosdep init   # only once per machine, skip if already done
  rosdep update
  ```
- **librealsense2 v2.51.1** for the RealSense T265. Newer librealsense releases dropped T265 support, and the `realsense_t265_shelfino` driver targets v2.51.1. Build it from source at tag `v2.51.1` (the Debian package is not available for Ubuntu 22.04 at that version).

## Installation

```bash
# 1. Create the workspace and clone this repository
mkdir -p ~/wcr_ws/src
cd ~/wcr_ws/src
git clone https://github.com/BCaran/wcr_stack.git -b humble

# 2. Import external dependencies (pinned in wcr.repos)
vcs import < wcr_stack/wcr.repos

# 3. Install system dependencies
cd ~/wcr_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# 4. Build
colcon build --symlink-install
source install/setup.bash
```

Add `source ~/wcr_ws/install/setup.bash` to your `~/.bashrc` to avoid sourcing it in every new terminal.

### Serial port permissions

The Dynamixel U2D2, the VESC and the Teensy are USB serial devices. Add your user to the `dialout` group, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## Usage

```bash
# Robot hardware + ros2_control controllers (pick the controller set you need)
ros2 launch wcr_bringup wcr_classic.launch.py

# Full system: hardware, sensors, teleop
ros2 launch wcr_system full_system.launch.py
```

Other controller configurations are available in `wcr_bringup/launch/` (`wcr_swerve`, `wcr_velocity`, `wcr_effort`, `wcr_smc`, `wcr_chained`, `wcr_mix`, `wcr_mimic`), with matching parameter files in `wcr_bringup/config/`. `mock_wcr_classic_full.launch.py` runs the stack without hardware.

## Repository layout

| Path | Contents |
|---|---|
| `wcr_bringup` | Launch files and ros2_control controller configurations |
| `wcr_controllers` | 4WIS4WID controllers: inverse kinematics, model-based and sliding-mode velocity/torque control, trajectory generator |
| `swerve_drive_controller` | ros2_control controller for the 4WIS4WID base: twist in, wheel velocities and steering position/velocity out, wheel odometry |
| `wheel_smc_controller` | Sliding-mode wheel controller as a ros2_control plugin |
| `wcr_odometry` | Wheel odometry for the 4WIS4WID platform |
| `wcr_interfaces` | Custom messages (`OdomAccel`, `ThrusterData`, `DesiredPoseTwist`, `LoadCellData`) |
| `wcr_system` | System-level launch files, sensor configs (T265, BNO055, OptiTrack, Marvelmind, VESC, PS4) and helper nodes |
| `wcr_teensy_bridge` | ROS 2 bridge to the Teensy microcontroller |
| `wcr_bench` | Test-bench node and configuration for force measurements |
| `firmware/` | Teensy firmware (not a ROS package) |
| `docker/` | Dockerfile and container entrypoint |
| `scripts/` | Setup, run, flashing scripts and udev rules |
| `wcr.repos` | Pinned external dependencies |

`firmware/`, `docker/` and `scripts/` contain an empty `COLCON_IGNORE` file so colcon skips them.

## External dependencies

| Repository | Version | Purpose |
|---|---|---|
| [f1tenth/vesc](https://github.com/f1tenth/vesc/tree/humble) | `humble` @ `1bc8251` | VESC driver for the EDF adhesion system |
| [ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers/tree/humble) | `humble` @ `d3f510c` | Serial transport used by the VESC driver |
| [idra-lab/realsense_t265_shelfino](https://github.com/idra-lab/realsense_t265_shelfino) | `ros2-master` @ `b8baca6` | RealSense T265 driver |
| [BCaran/dynamixel_hardware_interface](https://github.com/BCaran/dynamixel_hardware_interface) | `humble` @ `8d91886` | **Fork** of the ROBOTIS ros2_control interface for Dynamixel motors (see below) |
| [ROBOTIS-GIT/DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble) | `humble` @ `59abd67` | Dynamixel SDK |
| [ROBOTIS-GIT/dynamixel_interfaces](https://github.com/ROBOTIS-GIT/dynamixel_interfaces/tree/humble) | `humble` @ `9e2ea84` | Dynamixel message definitions |

### dynamixel_hardware_interface fork

The upstream [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) selects a model file from `param/dxl_model/` based on the model number read from each motor, and the location of these files cannot be configured. The fork changes the XH430-W210 model file so the effort interface reports torque instead of raw current values. This applies to every XH430-W210 on the bus. Controllers and estimators in this stack assume the converted torque values.

### Updating dependencies

```bash
cd ~/wcr_ws/src
vcs pull                                   # update all imported repos to their latest commits
# ... build and test ...
vcs export --exact > wcr_stack/wcr.repos   # freeze the tested state back into wcr.repos
```

Commit the updated `wcr.repos` together with any code changes that depend on it.

## Docker

Build from the repository root so the build context includes `wcr.repos` and the packages:

```bash
docker build -f docker/Dockerfile -t wcr_stack .
```

## Firmware

The Teensy firmware lives in `firmware/`. It shares its serial protocol with `wcr_teensy_bridge`, so update both when the protocol changes.