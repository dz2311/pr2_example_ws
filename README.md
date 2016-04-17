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
# Something about the perception
The frame_id in a message specifies the point of reference for data contained in that message.

For example, if I want to specify a goal that is 10 meters directly in front of my robot, I would first declare the "origin" of the robot, which I will name the /base_link frame. The origin of this /base_link frame (0,0,0) is located at the center of mass of my robot. I would send a goal of (10,0,0) in the /base_link frame.

In your scenario in which you have a sensor, the tf tree becomes slightly more complex. Let's say that I have a Kinect that's mounted at the very front of my robot, directly in front of the center of mass. I now declare an "origin" for that sensor and name it the /kinect frame. Since the sensor is mounted 0.25 meters in front of the center of mass, the transform (tf) from /base_link to /kinect is (0.25,0,0), or the transform from /kinect to /base_link is (-0.25,0,0) (because /base_link is 0.25 BEHIND the sensor).

Now is when tf becomes useful. Your sensor data arrives in the frame_id of the sensor collecting it (i.e. your kinect points arrive in the /kinect frame in my example). If the Kinect receives a point of 10.0 meters directly ahead, and you want to navigate to that point, your goal is (10,0,0) in the /kinect frame. However, remember that since your sensor is 0.25 meters IN FRONT of the robot's center, this goal is actually (10.25,0,0) in the /base_link frame. The beauty of tf is that it can transform points from the /kinect frame to the /base_link frame for you.

The only catch is that you must tell tf to perform this transform (you can learn to do this in the tf tutorials). If you're trying to specify a goal for move_base or some other ROS navigation software, chances are that this transform is done for you already (move_base definitely does this for you). This has to be determined on a case-by-case basis, since sometimes it is done for you, but sometimes it is not.

TL;DR: If you're trying to send a goal to move_base using the navigation stack, you don't need to change the frame of the point. If you collect the data in the /camera_rgb_optical_frame, then you can send a goal in the camera_rgb_optical_frame and the navigation stack will take care of the transform for you. 

# Something about TF
TF is only a representation of known data. It is used for queries like "I know the ball is 3m from the camera, and the camera is 2m next to my robot, where is the ball in regards to the robot?". You have to make sure yourself, that these informations are available and correct.

Now, for how to get this information: Writing everything from scratch is a lot of hard work. You said you already know, where your object is in your picture (I guess you have the information available as "x pixels from the left side and y pixels from the top"). Now you'll need some kind of depth information, like with a kinect where you can look up the distance at the area of your object. Or you know how big your target is, and then try to estimate the distance via the observed size of the object in your image. When you have this, then you can publish this information.

Or you use an already available package to do the work for you. Take a look at the ar_track_alvar package, it tracks markers via a kinect (or a normal webcam) and publishes this information as tf.

Maybe, if you can give a bigger picture of what you are trying to achieve, we can help you better.

Good luck

Next edit:

Regarding what you want to do: I was thinking along the lines of "What do you want to track the coloured object for? Could you use the ar_track_alvar package or some other instead?"

As I said, to write the complete code from "I have a picture with a red ball in the upper corner" to "The red ball is 2.5 x, 0.3 y and 1.2 z from the camera" is not just a couple of lines, especially if you have little experience / no idea where to get started. Try to find packages that already do the stuff you want to do and see if you can adapt them.

