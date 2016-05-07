#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>

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


void clearFile()
{
  std::ofstream ofs;
  ofs.open("/home/dz2311/PR2write.txt", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
}
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
**************************THis function is for writing points on  bezier curve into txt file **************************************
*/
void writeFile(float x, float y, float z, float basex, float basey)
{
  std::ofstream out("/home/dz2311/PR2write.txt", std::ios::app);
  if(out.good())
  {
	out << x-basex <<"\n";
	out << y-basey <<"\n";
	out << z <<"\n";
  }
  out.close();
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
  float delta_sample = 1.0/(float)(sample_time-1);

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
  float resol = 3.0;
  //int sample_time = atoi(argv[0]);
 // std::cout<< "Current resolution: "<<sample_time <<std::endl;

  ros::init(argc, argv, "write_letter");
  ros::NodeHandle node_handle;

  ros::ServiceClient modelstate = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  modelstate.waitForExistence();
  gazebo_msgs::GetModelState get_model;
  get_model.request.model_name = "pen";
  modelstate.call(get_model);
  float pen_position[3],initial_pen_position[3];
  //pen_position[0] = get_model.response.pose.position.x;
  //pen_position[1] = get_model.response.pose.position.y;
  //pen_position[2] = get_model.response.pose.position.z;
  initial_pen_position[0] = 1.8;
  initial_pen_position[1] = 0.4;
  initial_pen_position[2] = 0.55;
 
  RobotDriver driver(node_handle);
  ros::AsyncSpinner spinner(1);
  spinner.start();
   moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // move arm above the pen
  geometry_msgs::Pose start_pose,target_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = initial_pen_position[0]-0.050;     
  start_pose.position.y = initial_pen_position[1];
  start_pose.position.z = initial_pen_position[2]+0.23;
  group.setPoseTarget(start_pose); 
 // ROS_INFO("%f",start_pose.position.x);
 // ROS_INFO("%f",start_pose.position.y);
 // ROS_INFO("%f",start_pose.position.z);
  group.move();
  ROS_INFO("Start to write!!!");
  target_pose = start_pose;

  while(1)
  {
    ROS_INFO("Type the letter you wanna write! Type '1' to quit!");
    char c= getchar();
    getchar();
    if(c == '1') return 0;
    ROS_INFO("Start to write!");
    std::vector<float> cps;
    int line_num = (c-'a')+1;
    readFile(cps, line_num);
    /*int strokeN = cps.size()/9;

    std::vector<float> samples[strokeN];
    for(int i=0;i<strokeN;i++){
	for(int k =0;k<resol;k++){
	   float t = k * 1.0/(resol-1);
           float tempx = (1-t)*cps[i*9] + t*cps[i*9+3];
           float tempy = (1-t)*cps[i*9+1] + t*cps[i*9+4];
	   float tempxx = (1-t)*cps[i*9+3] + t*cps[i*9+6];
	   float tempyy = (1-t)*cps[i*9+4] + t*cps[i*9+7];
	   tempx = (1-t) * tempx + t*tempxx;
	   tempy = (1-t) * tempy + t*tempyy;
	   samples[i].push_back(tempx);
	   samples[i].push_back(tempy);
	}
    }
    for(int i=0; i < strokeN;++i){
	for(int j=0;j< samples[i].size()/2; j++)
      	    std::cout<< "(" << samples[i][j*2] << "," << samples[i][j*2+1] <<"),";
   	std::cout << ""<< std::endl;
    }
 
    for(int i=0; i<strokeN ;i++)
    {
	for(int j=0;j< samples[i].size()/2; j++){
       	  target_pose.position.x = start_pose.position.x + samples[i][j*2];
      	  target_pose.position.y = start_pose.position.y + samples[i][j*2+1];
      	  group.setPoseTarget(target_pose);
	  group.move();
	}
     }
  }*/
   int sample_time = 10;
    std::vector<float> out_points;
    bezierCurve(cps, out_points, sample_time);
    std::cout<<"bezier sampling size"<<out_points.size()<<std::endl;
    for(int i=0; i<(int)out_points.size();++i)
        std::cout<<out_points[i]<<std::endl;

    for(int i=0;i<(int)(out_points.size()/(sample_time*3));i++){
	target_pose.position.z = start_pose.position.z - 0.05;
	for(int k=0;k<sample_time;k++)
        {
         target_pose.position.x = start_pose.position.x + out_points[(sample_time*3)*i+3*k];
         target_pose.position.y = start_pose.position.y + out_points[(sample_time*3)*i+3*k+1];
         group.setPoseTarget(target_pose);
         modelstate.call(get_model);
         pen_position[0] = get_model.response.pose.position.x;
         pen_position[1] = get_model.response.pose.position.y;
         pen_position[2] = get_model.response.pose.position.z;
         writeFile(pen_position[0], pen_position[1], pen_position[2],  initial_pen_position[0], initial_pen_position[1]);
         group.move(); 
        }
	if(i+1<(out_points.size()/(sample_time*3))){
	    target_pose.position.z = start_pose.position.z;
	    target_pose.position.x = start_pose.position.x + out_points[(sample_time*3)*(i+1)];
            target_pose.position.y = start_pose.position.y + out_points[(sample_time*3)*(i+1)+1];
            group.setPoseTarget(target_pose);
            group.move(); 
        }
     }
      modelstate.call(get_model);
      pen_position[0] = get_model.response.pose.position.x;
      pen_position[1] = get_model.response.pose.position.y;
      pen_position[2] = get_model.response.pose.position.z;
      writeFile(pen_position[0], pen_position[1], pen_position[2],  initial_pen_position[0], initial_pen_position[1]);
      std::cout<<"Finish writing!Do you want to continue? y/n"<<std::endl;
      c = getchar();getchar();
      if(c=='n') break;
      clearFile();
      target_pose.position.x = start_pose.position.x;
      target_pose.position.y = start_pose.position.y;
      target_pose.position.z = start_pose.position.z;
      group.setPoseTarget(target_pose);
      group.move();
  }
    ROS_INFO("Finished writing!");
  return 0;
}
