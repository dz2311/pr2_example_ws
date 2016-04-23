#include <ros/ros.h>
#include <visualization_msgs/Marker.h> 
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
  ros::Rate r(1);

  while (ros::ok())
  {
    visualization_msgs::Marker points, line_strip, line_list;    //markers

    //marker - points, strip, list   information
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    // id
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    // three types
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;


    // Points are green, set color
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue, set color
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red, set color
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    ifstream infile;
    infile.open("/home/sz2558/A.txt");
    vector<float> v;
    string s;
    float value;

    while(!infile.eof()){
      infile>>s;
      value = atof(s.c_str()); 
      v.push_back(value);
      //cout<<value<<endl;
      s.clear();
    }

    infile.close();

    v.pop_back();
    int n = v.size();

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < n; i = i+3)
    {

      geometry_msgs::Point p;
      p.x = v[i]*5;
      p.y = v[i+1]*5;
      p.z = v[i+2]*5;

      // cout<<v[i]<<v[i+1]<<v[i+2]<<endl;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }


    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);

    //r.sleep();

    sleep(1);

  }
}
