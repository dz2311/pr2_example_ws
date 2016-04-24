#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
//#include <pr2_gripper_sensor_msgs/msg/PR2GripperForceServoAction.h>
#include <actionlib/client/simple_action_client.h>
class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(3.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 2.5 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 2.5;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      if(dist_moved-0.135 > distance) done = true;
    }
    if (done) return true;
    return false;
  }
};

class Gripper{
private:
  ros::NodeHandle nh_;
  ros::Publisher r_gripper_;
  tf::TransformListener listener_;

public:
  //Action client initialization

   
  Gripper(ros::NodeHandle &nh){
    nh_ = nh;
    r_gripper_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", 1);
  }

  ~Gripper(){
  }

  //Open the gripper
  void open(){

    ROS_INFO("Opening Gripper");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.09;
    gripper_cmd.max_effort = -1.0;

    ros::Rate rate(10.0);


    while (!done && nh_.ok())
    {
      r_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how if the gripper is open
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 0.04) done = true;
    
    }
  }

  //Close the gripper
  void close(){
    
    ROS_INFO("Closing Gripper");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.06;
    gripper_cmd.max_effort = 50.0;

    ros::Rate rate(10.0);

    double dist_moved_before;
    double dist_moved;
    while (!done && nh_.ok())
    {
      r_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
  
      //see how if the gripper is open or if it hit some object
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;

      dist_moved_before = dist_moved;
      dist_moved = relative_transform.getOrigin().length();

      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 1.0 || dist_moved < dist_moved_before) done = true;
    
    }
  }
};


int main(int argc, char *argv[])
{
  char letter_ = *argv[1];
  ros::init(argc, argv, "move_arm_example");
  ros::NodeHandle node_handle;
  RobotDriver driver(node_handle);
  Gripper gripper(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();
    driver.driveForwardOdom(1.0); //1.745,0.6 0.55
  moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
/*************************************************Add Collision****************************/
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();
  /* The id of the object is used to identify it. */
  collision_object.id = "table";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.05;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  2.15;
  box_pose.position.y =  0.0;
  box_pose.position.z =  0.275;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  planning_scene_interface.addCollisionObjects(collision_objects);
  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);
  /*std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);*/
  ROS_INFO("table added!");
/***************************************************************************************/  
  // right arm
  geometry_msgs::Pose goal_end_effector_pose;
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = 1.62;     
  goal_end_effector_pose.position.y = 0.6;
  goal_end_effector_pose.position.z = 0.72;
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();

  gripper.open();
  //ros::shutdown();

  std::cout<<"MOVEIT"<<std::endl;
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = 1.62;	
  goal_end_effector_pose.position.y = 0.6;
  goal_end_effector_pose.position.z = 0.6;
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();
  gripper.close();
  ros::Duration(3).sleep();
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = 1.62;	
  goal_end_effector_pose.position.y = 0.6;
  goal_end_effector_pose.position.z = 0.70;
  group.setPoseTarget(goal_end_effector_pose);
  group.move();

  //after grasp the pen, move the arm to another point of the table
    goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = 1.6;	
    goal_end_effector_pose.position.y = 0.4;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
 gripper.close();
 
   std::cout<<"ready to write"<<std::endl;
    //ready to write
  float basex = 1.6;
  float basey = 0.4;
  if(letter_=='a')
  {
 
   std::cout<<"input is a "<<std::endl;
 std::cout<<"111"<<std::endl;
    goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.02;	
    goal_end_effector_pose.position.y = basey + 0.02;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

    std::cout<<"222"<<std::endl;  ros::Duration(0.5).sleep();
    goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.04;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"333"<<std::endl; ros::Duration(0.5).sleep();
    goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.06;	
    goal_end_effector_pose.position.y = basey + 0.06;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"444"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.07;	
    goal_end_effector_pose.position.y = basey + 0.08;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

  std::cout<<"555"<<std::endl;  ros::Duration(0.5).sleep();  
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.06;	
    goal_end_effector_pose.position.y = basey + 0.10;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

  std::cout<<"666"<<std::endl;ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.12;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"777"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.02;	
    goal_end_effector_pose.position.y = basey + 0.14;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"888"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.06;	
    goal_end_effector_pose.position.y = basey + 0.10;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

  std::cout<<"999"<<std::endl;ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.10;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"101010"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.08;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"111111"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.06;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();

 std::cout<<"121212"<<std::endl; ros::Duration(0.5).sleep();
     goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.04;	
    goal_end_effector_pose.position.y = basey + 0.04;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
  }
  return 0;
}
