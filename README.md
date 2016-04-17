# COMSE 6731 HUMANOID ROBOTS, SPRING 2016
# Columbia University


## Getting Started

The following lines will not work unless ROS is properly installed.  You can find instructions for how to do this from the ros_tutorial pdf on the class website.

```bash
$ cd ~
$ git clone git@github.com:HumanoidRobotics/pr2_example_code.git
$ cd pr2_example_code
$ source /opt/ros/indigo/setup.bash
$ catkin_make
$ source devel/setup.bash
```

## Running the Demo code
First, bring up Gazebo, Moveit and the PR2
```bash
$ export KINECT1=true
$ roslaunch system_launch pr2_gazebo_moveit.launch
```
Note: running `export KINECT1=true` allows Gazebo to know you will be using the kinect model.

Then run the individual demos with any of the following:
```bash
$ rosrun move_arm move_arm
$ rosrun move_base move_base
$ rosrun move_gripper move_gripper
$ rosrun move_head move_head
```
