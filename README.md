# wcr_stack
Stack for Wall climbing robot with 4WIS4WID kinematic structure

# Getting started
```
mkdir -p wcr/src
cd wcr/src
git clone https://github.com/BCaran/wcr_stack.git -b humble
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```
IMPORTANT! \
Before launching Dynamixel hardware interface, permission has to be added to USB port
```
sudo usermod -aG dialout <linux_account>
```
# Packages
## _wcr_description_
_urdf_ files and _ros2_control_ xacro files for Dynamixel motors. To visulize robot, only _display.launch.py_ should be used.

## _dynamixel-workbench_ and _dynamixel-workbench-msgs_ 
Used for communication with driving and steering Dynamixel motors

## _dynamixel_hardware_
Used as _ros2_control_ layer. \
Problem: current problem is that layer isn't supporting both position and velocity control modes for Dynamixel motors at once.

## _vesc_ and _transport_drivers_
Used for communication with VESC that is controlling EDF addhesion system.

#TO-DO LIST
- [ ] 4WIS4WID controller using _dynamixel-workbench_
- [x] package for controlling thruster motors (Teensy communication)
- [ ] work on _dynamixel_hardware_ to be able to have both position and velocity control modes
- [ ] 4WIS4WID controller using _ros2_control_ framework
- [ ] 
