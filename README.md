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

## _dynamixel-workbench_ and _dynamixel-workbench-msgs_ 
Used for communication with driving and steering Dynamixel motors

## dynamixel_hardware
Used as _ros2_control_ layer, but current problem is that layer isn't supporting both position and velocity control modes at once.

# IMPORTANT!
Before launching Dynamixel hardware interface, permission has to be added to USB port
```
sudo usermod -aG dialout <linux_account>
```
