#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
//#include <pr2_gripper_sensor_msgs/msg/PR2GripperForceServoAction.h>
#include <gazebo_msgs/GetModelState.h>
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

void writeFIle_StraightLine(float x_prev, float y_prev, float x_cur, float y_cur)
{
  int sample = 10;
  std::ofstream out("/home/dz2311/PR2write.txt", std::ios::app);
  float arr[sample*2+4];
  float disx = (x_cur-x_prev)/(sample+1);
  float disy = (y_cur-y_prev)/(sample+1);
  for(int i=0; i<sample+2;++i)
  {
    arr[i] = x_cur+i*disx;
    arr[i+1] = y_cur+i*disy;
    if(out.good())
      {
	out << arr[i] <<"\n";
	out << arr[i+1] <<"\n";
      }
  }
  out.close();
}

int main(int argc, char *argv[])
{
  char letter_ = *argv[1];
  ros::init(argc, argv, "move_arm_example");
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
    driver.driveForwardOdom(pen_position[0]-0.85); //1.745,0.6 0.55
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
   geometry_msgs::Pose goal_end_effector_pose;
   std::cout<<"ready to write"<<std::endl;
    //ready to write geometry_msgs::Pose goal_end_effector_pose;
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

  float basex = goal_end_effector_pose.position.x;
  float basey = goal_end_effector_pose.position.y;
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
    writeFIle_StraightLine(basex, basey,  goal_end_effector_pose.position.x,  goal_end_effector_pose.position.y);
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

  if(letter_=='b')
  {
     std::cout<<"input is b "<<std::endl;
     std::cout<<"111"<<std::endl;
    goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = basex + 0.1;	
    goal_end_effector_pose.position.y = basey;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
   
    float vx = basex+0.1;
    float vy = basey;
    int ii = 0;
    while(vx>=basex)
    {
      vx-=0.05;
      if(ii<5||(ii>10&&ii<15))
	vy-=0.1;
      else
	vy+=0.1;
       std::cout<<ii<<std::endl;
      goal_end_effector_pose.orientation.w = 1.0;
    goal_end_effector_pose.position.x = vx;	
    goal_end_effector_pose.position.y = vy;
    goal_end_effector_pose.position.z = 0.65;
    group.setPoseTarget(goal_end_effector_pose);
    group.move();
    ii++;
    }
}
  return 0;
}
