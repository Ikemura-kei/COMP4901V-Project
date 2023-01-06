#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <hkust_rgd_msgs/dynamic_obstacles.h>
#include <cstdlib>
#define NUM_OBSTACLES 7
#define INIT_POS_DIST 0.959 // in m
#define CONST_VEL_X 0.3    // in m/s
#define CONST_VEL_Y 0.3    // in m/s
#define OMEGA 0.7        // in rad/s
#define RADIUS 0.756       // in m, the motion radius
#define RATE 50            // in Hz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_obstacle_gen_node");

    ros::NodeHandle nh;

    ros::Publisher obstaclePub = nh.advertise<hkust_rgd_msgs::dynamic_obstacles>("dynamic_obstacles", 100);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::MarkerArray>("dynamic_obstacle_marker", 100);

    hkust_rgd_msgs::dynamic_obstacles obstacles;
    visualization_msgs::Marker obstacleMarkers[NUM_OBSTACLES];
    int dirsX[NUM_OBSTACLES];
    int dirsY[NUM_OBSTACLES];
    float radi[NUM_OBSTACLES];
    float omegas[NUM_OBSTACLES];
    visualization_msgs::MarkerArray obstacleMarkerArray;

    for (int i = 0; i < NUM_OBSTACLES; i++)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = CONST_VEL_X * (i % 2 == 0 ? -1 : 1);
        twist.linear.y = CONST_VEL_Y * (i % 2 == 0 ? 1 : -1);

        geometry_msgs::Pose pose;
        pose.position.x = i * INIT_POS_DIST * (rand() % 3 == 0 ? 1 : -1) + (rand() % 10) * 0.05;
        pose.position.y = i * INIT_POS_DIST * (rand() % 5 == 0 ? 1 : -1) + (rand() % 10) * 0.02;
        pose.position.z = 0;

        obstacles.twist.push_back(twist);
        obstacles.pose.push_back(pose);

        dirsX[i] = (rand() % 2 == 0 ? 1 : -1);
        dirsY[i] = (rand() % 2 == 0 ? 1 : -1);
        float factor = (rand() % 9 + 1) * 0.1 + 1;
        radi[i] = factor * RADIUS;
        omegas[i] = 1 / factor * OMEGA; // larger the radius, smaller the omega

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
    ros::Rate rate(RATE); // in Hz
    while (ros::ok())
    {
        cnt++;
        obstaclePub.publish(obstacles);
        markerPub.publish(obstacleMarkerArray);

        obstacleMarkerArray.markers.clear();
        for (int i = 0; i < NUM_OBSTACLES; i++)
        {
            if (cnt * 1.0 / RATE > 2.0)
            {
                obstacles.twist[i].linear.x = radi[i] * cos(cnt * 1.0 / RATE * omegas[i] * dirsX[i]);
                obstacles.twist[i].linear.y = radi[i] * sin(cnt * 1.0 / RATE * omegas[i] * dirsY[i]);
            }
            obstacles.pose[i].position.x = obstacles.pose[i].position.x + 1.0 / RATE * obstacles.twist[i].linear.x;
            obstacles.pose[i].position.y = obstacles.pose[i].position.y + 1.0 / RATE * obstacles.twist[i].linear.y;

            obstacleMarkers[i].pose.position.x = obstacles.pose[i].position.x;
            obstacleMarkers[i].pose.position.y = obstacles.pose[i].position.y;
            obstacleMarkers[i].pose.position.z = obstacles.pose[i].position.z;
            obstacleMarkerArray.markers.push_back(obstacleMarkers[i]);
        }

        rate.sleep();
    }
}
