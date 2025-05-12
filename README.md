# wcr_stack
Stack for Wall climbing robot with 4WIS4WID kinematic structure

# Getting started on companion PC
```
mkdir -p wcr_pc/src
cd wcr/src
git clone https://github.com/BCaran/wcr_stack.git
```
On PC we only need _urdf_ of the robot for visualization in _rviz2_
```
cd ..
colcon build --packages-select wcr_description
source install/setup.bash
```
For robot visualization and _urdf_ check we use only _robot_state_publisher_, _joint_state_publisher_gui_ and robot's _urdf_
```
ros2 launch wcr_description display_dummy.launch.py
```
<img
  src="https://github.com/user-attachments/assets/294211be-3e27-4112-898b-1a929239729b"
  alt="Robot URDF visualization"
  width="640"
  height="360"
/>

**IF THE ROBOT IS RUNNING** we can use following _.launch_ to display robot's states.
```
ros2 launch wcr_description display.launch.py
```
# Getting started on _wcr_
```
mkdir -p wcr/src
cd wcr/src
git clone https://github.com/BCaran/wcr_stack.git -b humble
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```
**IMPORTANT!** \
Before following commands, make sure that permissions have been added to the USB ports for _Dynamixel_, _VESC_ and _Teensy_

System launching starts nodes for driving robot and odometry, IMU (_BNO055_ or _olixSense™ IMU_), _RealSense T265_ and _PS4_ joystick.

