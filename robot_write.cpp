#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>

class RobotDriver
{
  private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    tf::TransformListener listener_;
  public:
    RobotDriver(ros::NodeHandle &nh)
    {
      nh_ = nh;
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
    }

    bool driveForwardOdom(double distance)
   {
      ros::Duration(0.5).sleep();
      listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(3.0));
    
      tf::StampedTransform start_transform;
      tf::StampedTransform current_transform;

      //record the starting transform from the odometry to the base frame
      listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
      geometry_msgs::Twist base_cmd;
      //the command will be to go forward at 2.5 m/s
      base_cmd.linear.y = base_cmd.angular.z = 0;
      base_cmd.linear.x = 2.5;
    
      ros::Rate rate(10.0);
      bool done = false;
      while (!done && nh_.ok())
      {
        cmd_vel_pub_.publish(base_cmd);
        rate.sleep();
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
        if(dist_moved > distance) done = true;
      }
      if (done) return true;
      return false;
    }
};

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.09;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    sleep(5);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.01;
    squeeze.command.max_effort = -1.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    //sleep(10);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_write");
  ros::NodeHandle node_handle;

  ros::ServiceClient modelstate = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  modelstate.waitForExistence();
  gazebo_msgs::GetModelState get_model;
  get_model.request.model_name = "coke";
  modelstate.call(get_model);
  float pen_position[3];
  ROS_INFO("%f",get_model.response.pose.position.x);
  ROS_INFO("%f",get_model.response.pose.position.y);
  ROS_INFO("%f",get_model.response.pose.position.z);
  pen_position[0] = get_model.response.pose.position.x;
  pen_position[1] = get_model.response.pose.position.y;
  pen_position[2] = get_model.response.pose.position.z;
 
  RobotDriver driver(node_handle);
  Gripper gripper;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  driver.driveForwardOdom(pen_position[0]-0.85); 
  //driver.driveForwardOdom(0.1); 

  moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

/*  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();
  collision_object.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.05;

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

  sleep(2.0);

  ROS_INFO("table added!");*/
/***************************************************************************************/  
  // move arm above the pen
  geometry_msgs::Pose goal_end_effector_pose;
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = pen_position[0]-0.18;     
  goal_end_effector_pose.position.y = pen_position[1];
  goal_end_effector_pose.position.z = pen_position[2]+0.18;
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();
  ROS_INFO("ABOVE THE PEN");
  gripper.open();
  
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = pen_position[0]-0.18;	
  goal_end_effector_pose.position.y = pen_position[1];
  goal_end_effector_pose.position.z = pen_position[2]+0.01;
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();
  
  gripper.close();
  sleep(30);

  ROS_INFO("Which character to write?"); 

  ROS_INFO("sTART TO WRITE");
  float start_position[3] = {goal_end_effector_pose.position.x,goal_end_effector_pose.position.y,goal_end_effector_pose.position.z};
  for(int i=1;i<5;i++)
  { 
    goal_end_effector_pose.position.x = start_position[0]+0.05;	
    goal_end_effector_pose.position.y = start_position[1];
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
    goal_end_effector_pose.position.x = start_position[0]+0.05;	
    goal_end_effector_pose.position.y = start_position[1]+0.05;
    group.setPoseTarget(goal_end_effector_pose);
    group.move(); 
    goal_end_effector_pose.position.x = start_position[0];	
    goal_end_effector_pose.position.y = start_position[1]+0.05;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
    goal_end_effector_pose.position.x = start_position[0];	
    goal_end_effector_pose.position.y = start_position[1];
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
  }
 
  return 0;
}
