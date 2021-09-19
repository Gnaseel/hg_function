#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <math.h>
#include "hg_obstacle_detection/ParkPoint.h"
#include "hg_obstacle_detection/msgVelodyne.h"

using namespace std;
visualization_msgs::Marker points, line_strip;

float changeR=0;
float changeG=0;
float changeB=0;

void color_mark(const hg_obstacle_detection::ParkPoint::ConstPtr& msg){

  line_strip.points.clear();
  for(int i = 0 ; i< msg->park_points.size()+1; i++){
    geometry_msgs::Point p;
    if(i<msg->park_points.size()){
      p.x = msg->park_points[i].x;
      p.y = msg->park_points[i].y;
      p.z = 0.0;
    }
    else if(i == msg->park_points.size()){
      p.x = msg->park_points[0].x;
      p.y = msg->park_points[0].y;
      p.z = 0.0;      
    }
    line_strip.points.push_back(p);
  }
  if(msg->can_park.data){
    changeR=0.53f;
    changeG=0.81f;
    changeB=0.98f;
  }
  else{
    changeR=1.0f;
    changeG=0.0f;
    changeB=0.0f;
  }
}
visualization_msgs::Marker init(){
  visualization_msgs::Marker line_strip;
    line_strip.ns = "lines";
    line_strip.header.frame_id = "/velodyne";
    line_strip.header.stamp = ros::Time::now();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.lifetime = ros::Duration(0.1);
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.pose.orientation.w = 0.0;
    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.color.a = 0.7f;
    return line_strip;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "velodyne_zone");
  ros::NodeHandle nh;
  ros::Subscriber subMin = nh.subscribe("/park_point", 10, color_mark);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/parking_marker", 10);
  ros::Rate loop_rate(50);
  line_strip = init();
  while (ros::ok()){
    
    line_strip.color.r = changeR;
    line_strip.color.g = changeG;
    line_strip.color.b = changeB;

    marker_pub.publish(line_strip);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}