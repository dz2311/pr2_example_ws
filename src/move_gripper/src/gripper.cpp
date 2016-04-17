#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class Gripper{
private:
  ros::NodeHandle nh_;
  ros::Publisher r_gripper_;
  tf::TransformListener listener_;

public:
  //Action client initialization
  Gripper(ros::NodeHandle &nh){
    nh_ = nh;
    r_gripper_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/r_gripper_controller/command", 1);
  }

  ~Gripper(){
  }

  //Open the gripper
  void open(){

    ROS_INFO("Opening Gripper");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
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
        listener_.lookupTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
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
    listener_.waitForTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "r_gripper_l_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.0;
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
      if(dist_moved > 0.03 || dist_moved < dist_moved_before) done = true;
    
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");
  ros::NodeHandle nh;

  Gripper gripper(nh);

  gripper.open();
  gripper.close();

  return 0;
}
