#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

ros::Publisher *transformedCloudPointPub;
ros::Publisher transformedCloudPointPub_;
static sensor_msgs::PointCloud2 transformedPointCloud;
void rawPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

transformedPointCloud = *msg;

transformedPointCloud.header.frame_id = "adjusted_camera_link";

    transformedCloudPointPub->publish(transformedPointCloud);
}

int main(int argc, char** argv)
{
ros::init(argc, argv,  "transform_cloud_point" );

ros::NodeHandle nh;

transformedCloudPointPub_ = nh.advertise<sensor_msgs::PointCloud2>("tranformed_cloud_point", 100);
transformedCloudPointPub = &transformedCloudPointPub_;

ros::Subscriber rawCloudPointSub = nh.subscribe("/orb_slam3/all_points", 1000, rawPointCloudCb);

while(ros::ok())
{
    ros::spinOnce();
    ros::Rate(1).sleep();
}
}