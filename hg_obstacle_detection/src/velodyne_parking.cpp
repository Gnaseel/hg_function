#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "hg_obstacle_detection/ParkPoint.h"
#include "std_msgs/Bool.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <iostream>

using namespace std;

class MyPoint{
public:
    float X;
    float Y;
    MyPoint(float x, float y);
    void setCoord(float x, float y);
    int ccw(const MyPoint& a, const MyPoint& b, const MyPoint& obs);
};

double minX=0;
double minY=0;
double minZ=0;
double maxX=0;
double maxY=0;
double maxZ=0;
double P_first_x=0;
double P_first_y=0;
double P_second_x=0;
double P_second_y=0;
double P_third_x=0;
double P_third_y =0;
double P_fourth_x=0;
double P_fourth_y=0;
ros::Publisher mark_pub;
ros::Publisher pub_flag;
std_msgs::Bool flag;
hg_obstacle_detection::ParkPoint mark_data;

MyPoint::MyPoint(float x, float y){ X=x; Y=y; }
void MyPoint::setCoord(float x, float y){ X=x; Y=y; }
int ccw(const MyPoint& a, const MyPoint& b, const MyPoint& obs){
    float vec = (b.X - a.X) * (obs.Y - a.Y) - (b.Y - a.Y) * (obs.X - a.X);
    if(vec < 0)      return -1;
    else if(vec > 0) return 1;
    else             return 0;
}
void downSampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output){
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> VG;
    VG.setInputCloud(cloud);
    VG.setLeafSize(0.1f, 0.1f, 0.1f);
    VG.filter(*output);
}
void ROI_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output){
  pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
  cropFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0)); // x, y, z, min (m)
  cropFilter.filter(*output);
}
void pc_callback(const sensor_msgs::PointCloud2& msg){
    sensor_msgs::PointCloud2 output = msg;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr total (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(output, *cloud);

    ROI_filter(cloud, total);
    // downSampling(total, total);

    MyPoint p1(P_first_x, P_first_y);
    MyPoint p2(P_second_x, P_second_y);
    MyPoint p3(P_third_x, P_third_y);
    MyPoint p4(P_fourth_x, P_fourth_y);
    // cout<<P_first_x<<" "<<P_first_y<<endl;
    // cout<<P_second_x<<" "<<P_second_y<<endl;
    // cout<<P_third_x<<" "<<P_third_y<<endl;
    // cout<<P_first_x<<" "<<P_first_y<<endl;

    bool detected=false;
    MyPoint obs(0,0);
    int count=0;
    // cout<<"SIZE = "<<total->points.size()<<endl;
    for(int i = 0; i < total->points.size(); ++i){
        obs.setCoord(total->points[i].x, total->points[i].y);
        count = 0;
        count += ccw(p1, p2, obs);
        count += ccw(p2, p3, obs);
        count += ccw(p3, p4, obs);
        count += ccw(p4, p1, obs);
        if(count == 4) detected=true;
    }
    if(detected){
      mark_data.can_park.data = false;
      flag.data=false;
    }else{
      mark_data.can_park.data = true;
      flag.data=true;
    }
    pub_flag.publish(flag);
}
hg_obstacle_detection::ParkPoint setPoint(ros::NodeHandle nh){

    nh.getParam("minX",minX);
    nh.getParam("minY",minY);
    nh.getParam("minZ",minZ);

    nh.getParam("maxX",maxX);
    nh.getParam("maxY",maxY);
    nh.getParam("maxZ",maxZ);

    nh.getParam("x1",P_first_x);
    nh.getParam("x2",P_second_x);
    nh.getParam("x3",P_third_x);
    nh.getParam("x4",P_fourth_x);

    nh.getParam("y1",P_first_y);
    nh.getParam("y2",P_second_y);
    nh.getParam("y3",P_third_y);
    nh.getParam("y4",P_fourth_y);

    hg_obstacle_detection::ParkPoint mark_data;
    geometry_msgs::Point park_first;
    geometry_msgs::Point park_second;
    geometry_msgs::Point park_third;
    geometry_msgs::Point park_fourth;
    park_first.x = P_first_x;
    park_second.x = P_second_x;
    park_third.x = P_third_x;
    park_fourth.x = P_fourth_x;

    park_first.y = P_first_y;
    park_second.y = P_second_y;
    park_third.y = P_third_y;
    park_fourth.y = P_fourth_y;
    mark_data.park_points.push_back(park_first);
    mark_data.park_points.push_back(park_second);
    mark_data.park_points.push_back(park_third);
    mark_data.park_points.push_back(park_fourth);
    return mark_data;
}
int main(int argc, char ** argv){
    
    ros::init(argc, argv, "velodyne_parking");
    ros::NodeHandle nh;

    ros::Subscriber subPoint = nh.subscribe("/velodyne_points", 50, pc_callback);
    mark_pub = nh.advertise<hg_obstacle_detection::ParkPoint>("/park_point", 10);
    pub_flag = nh.advertise<std_msgs::Bool>("/parking_obs_flag", 10);
    ros::Rate r(60);

    mark_data = setPoint(nh);


    while(ros::ok()){
        mark_pub.publish(mark_data);
        ros::spinOnce();
        r.sleep();
    }
}
