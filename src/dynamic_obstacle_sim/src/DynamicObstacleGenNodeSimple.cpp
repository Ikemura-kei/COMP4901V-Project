#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <hkust_rgd_msgs/dynamic_obstacles.h>
#include <cstdlib>
#define NUM_OBSTACLES 12
#define INIT_POS_DIST 0.359 // in m
#define CONST_VEL_X 0.25    // in m/s
#define CONST_VEL_Y 0.11   // in m/s
#define RATE 50            // in Hz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_obstacle_gen_node_simple");

    ros::NodeHandle nh;

    ros::Publisher obstaclePub = nh.advertise<hkust_rgd_msgs::dynamic_obstacles>("dynamic_obstacles", 100);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::MarkerArray>("dynamic_obstacle_marker", 100);

    hkust_rgd_msgs::dynamic_obstacles obstacles;
    visualization_msgs::Marker obstacleMarkers[NUM_OBSTACLES];
    int dirsX[NUM_OBSTACLES];
    int dirsY[NUM_OBSTACLES];
    visualization_msgs::MarkerArray obstacleMarkerArray;

    for (int i = 0; i < NUM_OBSTACLES; i++)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = CONST_VEL_X + (rand() % 10) / 45.0f;
        twist.linear.y = CONST_VEL_Y;

        geometry_msgs::Pose pose;
        pose.position.x = i * INIT_POS_DIST * (rand() % 3 == 0 ? 1 : -1) + (rand() % 10) * 0.05;
        pose.position.y = i * INIT_POS_DIST * (rand() % 5 == 0 ? 1 : -1) + (rand() % 10) * 0.02;
        pose.position.z = 0;

        obstacles.twist.push_back(twist);
        obstacles.pose.push_back(pose);

        dirsX[i] = (rand() % 2 == 0 ? 1 : -1);
        dirsY[i] = (rand() % 2 == 0 ? 1 : -1);
        float factor = (rand() % 9 + 1) * 0.1 + 1;

        obstacleMarkers[i].header.frame_id = "map";
        obstacleMarkers[i].header.stamp = ros::Time();
        obstacleMarkers[i].ns = "dynamic_obstacle";
        obstacleMarkers[i].id = i;
        obstacleMarkers[i].type = visualization_msgs::Marker::SPHERE;
        obstacleMarkers[i].action = visualization_msgs::Marker::ADD;
        obstacleMarkers[i].pose.position.x = pose.position.x;
        obstacleMarkers[i].pose.position.y = pose.position.y;
        obstacleMarkers[i].pose.position.z = pose.position.z;
        obstacleMarkers[i].pose.orientation.x = 0.0;
        obstacleMarkers[i].pose.orientation.y = 0.0;
        obstacleMarkers[i].pose.orientation.z = 0.0;
        obstacleMarkers[i].pose.orientation.w = 1.0;
        obstacleMarkers[i].scale.x = 0.171;
        obstacleMarkers[i].scale.y = 0.171;
        obstacleMarkers[i].scale.z = 0.09;
        obstacleMarkers[i].color.a = 1.0; // Don't forget to set the alpha!
        obstacleMarkers[i].color.r = 0.3;
        obstacleMarkers[i].color.g = 0.6;
        obstacleMarkers[i].color.b = 0.9;
    }

    int cnt = 0;          // keep tracks of the loop count
    int dir = 1;
    ros::Rate rate(RATE); // in Hz
    while (ros::ok())
    {
        cnt++;
        obstaclePub.publish(obstacles);
        markerPub.publish(obstacleMarkerArray);

        obstacleMarkerArray.markers.clear();
        if(cnt % 955 == 0)
        {
            dir *= -1;
        }
        for (int i = 0; i < NUM_OBSTACLES; i++)
        {
            obstacles.pose[i].position.x = obstacles.pose[i].position.x + dir * 1.0 / RATE * obstacles.twist[i].linear.x;
            obstacles.pose[i].position.y = obstacles.pose[i].position.y + dir * 1.0 / RATE * obstacles.twist[i].linear.y;

            obstacleMarkers[i].pose.position.x = obstacles.pose[i].position.x;
            obstacleMarkers[i].pose.position.y = obstacles.pose[i].position.y;
            obstacleMarkers[i].pose.position.z = obstacles.pose[i].position.z;
            obstacleMarkerArray.markers.push_back(obstacleMarkers[i]);
        }

        rate.sleep();
    }
}
