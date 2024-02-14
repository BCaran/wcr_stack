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
