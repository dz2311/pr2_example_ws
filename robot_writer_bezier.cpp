#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
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

/*
**************************THis function is for reading txt file**************************************
*/

void readFile(std::vector<float> &points, int spec_line)
{
  points.clear();
  std::ifstream infile;
  infile.open ("/home/dz2311/letters.txt");
  char buffer[1025];
  
  for(int line=1; infile.good();line++)
  {
    infile.getline(buffer, 1024);
    buffer[infile.gcount()]=0;

    if(line==spec_line)
    {
      std::istringstream iss(buffer);
      while(!iss.eof())
	{
	  float trs = 0.0;
	  iss >> trs;
	  points.push_back(trs);
	}
      break;
    }
 }
  for(int i=0; i<(int)points.size();++i)
    std::cout<<points[i]<<std::endl;
  infile.close();
}

/*
**************************THis function is for calculating bezier curve**************************************
*/
void interpolation(float x, float y, float z, float x1, float y1, float z1, float x2, float y2, float z2,float t, std::vector<float> &output_vec)
{
    float outpointx1 = (1-t)*x+t*x1;
    float outpointy1 = (1-t)*y+t*y1;
    float outpointz1 = (1-t)*z+t*z1;

    float outpointx2 = (1-t)*x1+t*x2;
    float outpointy2 = (1-t)*y1+t*y2;
    float outpointz2 = (1-t)*z1+t*z2;

    float outpointx = (1-t)*outpointx1 + t*outpointx2;
    float outpointy = (1-t)*outpointy1 + t*outpointy2;
    float outpointz = (1-t)*outpointz1 + t*outpointz2;
    output_vec.push_back(outpointx);
    output_vec.push_back(outpointy);
    output_vec.push_back(outpointz);
  
}
  void bezierCurve(const std::vector<float> in_points, std::vector<float> &out_points, int sample_time)
{
  int row_offset = (int)in_points.size()/9;
  float delta_sample = 1/(float)(sample_time-1);

   for(int i=0; i<row_offset; ++i)
   {
       for(float t = 0.0; t<=1.0;t+=delta_sample)
      {
	std::vector<float> output_vec;
	interpolation(in_points[9*i],in_points[9*i+1],in_points[9*i+2],in_points[9*i+3],in_points[9*i+4],in_points[9*i+5], in_points[9*i+6],in_points[9*i+7],in_points[9*i+8], t, out_points);
      }
 }
}

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

  /*moveit_msgs::CollisionObject collision_object;
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

  sleep(10.0);

  ROS_INFO("table added!");*/
  
/***************************************************************************************/  
  // move arm above the pen
  geometry_msgs::Pose goal_end_effector_pose;
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = pen_position[0]-0.178;     
  goal_end_effector_pose.position.y = pen_position[1];
  goal_end_effector_pose.position.z = pen_position[2]+0.18;

  ROS_INFO("%f", goal_end_effector_pose.position.x);
  ROS_INFO("%f", goal_end_effector_pose.position.y);
  ROS_INFO("%f", goal_end_effector_pose.position.z);
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();
  ROS_INFO("ABOVE THE PEN");
  gripper.open();
  
  goal_end_effector_pose.orientation.w = 1.0;
  goal_end_effector_pose.position.x = pen_position[0]-0.178;	
  goal_end_effector_pose.position.y = pen_position[1];
  goal_end_effector_pose.position.z = pen_position[2]+0.02;
  ROS_INFO("%f", goal_end_effector_pose.position.x);
  ROS_INFO("%f", goal_end_effector_pose.position.y);
  ROS_INFO("%f", goal_end_effector_pose.position.z);
  group.setPoseTarget(goal_end_effector_pose); 
  group.move();
  
  gripper.close();
  sleep(15);
  ROS_INFO("Which character to write?"); 

  geometry_msgs::Pose target_pose = goal_end_effector_pose;
  target_pose.position.z += 0.04;
  group.setPoseTarget(target_pose);
  group.move();
  target_pose.position.x += 0.1;
  group.setPoseTarget(target_pose);
  group.move();

/*  for(int i=0;i<3;i++)
  {
     target_pose.position.z += 0.02;
     waypoints.push_back(target_pose);
  }
     target_pose.position.x += 0.1;
     waypoints.push_back(target_pose);

  for(int i=0;i<waypoints.size();i++){
     group.setPoseTarget(waypoints[i]);
     std::cout<<waypoints[i].position.z<<std::endl;
     group.move();
  }*/
  float start_position[3] = {target_pose.position.x,target_pose.position.y,target_pose.position.z};
  // std::cout<<start_position[2]<<std::endl;

  float offset[5][2] = {{-0.2,0.1},{0,0},{-0.2,-0.1},{-0.05,0.05},{-0.05,-0.05}};
  int order[5] ={0,1,2,4,3};
  
  while(1)
  {
    char c= getchar();
    getchar();
    if(c == '1') return 0;
    ROS_INFO("Start to write!");
    std::vector<float> points;
    int line_num = (c-'a')+1;
    std::cout<<line_num<<std::endl;
    readFile(points, line_num);
    std::cout<<points.size()<<std::endl;
    for(int i=0; i<(int)points.size();++i)
      std::cout<<points[i]<<std::endl;
    
    int sample_time = 5;
    std::vector<float> out_points;
    bezierCurve(points, out_points, sample_time);
     std::cout<<out_points.size()<<std::endl;
    for(int i=0; i<(int)out_points.size();++i)
        std::cout<<out_points[i]<<std::endl;
    for(int i=0;i<(int)(out_points.size()/3);i++)
    {
      target_pose.position.x = start_position[0]+out_points[3*i];
      target_pose.position.y = start_position[1]+out_points[3*i+1];
      group.setPoseTarget(target_pose);

      group.move(); 
     }
    /*target_pose.position.x = start_position[0]-0.1;	
    target_pose.position.y = start_position[1]-0.1;
    
    group.setPoseTarget(target_pose);
    group.move(); 
    target_pose.position.x = start_position[0]-0.1;	
    target_pose.position.y = start_position[1]+0.1;
    group.setPoseTarget(target_pose);
    group.move();
    target_pose.position.x = start_position[0];	
    target_pose.position.y = start_position[1];
    group.setPoseTarget(target_pose);
    group.move();*/
    //sleep(2);
    ROS_INFO("Finished writing!");
 }
  return 0;
}
