import moveit_commander
import numpy as np
import rospy

#initial a ros node
rospy.init_node("moveit_demo_node")

#get the robot
robot  = moveit_commander.RobotCommander()

#lets plan some trajectories for the left arm
group = robot.get_group('left_arm')

#generate a random pose for the end effector
pose = group.get_random_pose()

#generate a trajectory to get the gripper from
#its current location to this new pose
trajectory = group.plan(pose)


print (
    """
import moveit_commander
import numpy as np
import rospy

#initial a ros node
rospy.init_node("moveit_demo_node")

#get the robot
robot  = moveit_commander.RobotCommander()

#lets plan some trajectories for the left arm
group = robot.get_group('left_arm')

#generate a random pose for the end effector
pose = group.get_random_pose()

#generate a trajectory to get the gripper from
#its current location to this new pose
trajectory = group.plan(pose)
    """)

import IPython
IPython.embed()
group.execute(trajectory)
