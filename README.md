# EX1 joystick operation in Omniverse Isaac-Sim

## Build
```
cd {your_ws}/src
git clone {this repo}
cd ..
rosdep install --from-paths src -yi
mkdir build devel logs
catkin build omni_ex1_joy
```
## Topic
`ex1_teleop_joy.launch`: \
Subscribe to `joy` topic \
Publish `steeering_command` and `drive_command`

`ex1_ackermann_teleop.launch`: \
Subscribe to `joy` topic \
Publish `ackermann_steering_cmd` topic \
Subscribe to `ackermann_steering_cmd` topic \
Publish `steeering_command` and `drive_command`

## Operation

1. Naiive control (joy inclination is linearly converted to steering angle.)
```bash
roslaunch omni_ex1_joy ex1_teleop_joy.launch
```

2. Ackermann steering control
```bash
roslaunch omni_ex1_joy ex1_ackermann_teleop.launch
```