#include <ros/ros.h>
#include <hkust_rgd_msgs/voice_cast.h>
#include <hkust_rgd_msgs/rgd_command.h>
#include <hkust_rgd_msgs/gestures.h>
#include <iostream>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <robot_message/VoiceReco.h>
#include <robot_message/SLU.h>
#include <robot_message/voice_cast.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <hkust_rgd_behavior/Actions.hpp>
#include <hkust_rgd_behavior/Targets.hpp>
#include <hkust_rgd_behavior/Locations.hpp>

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return (buf);
}

static const float NAV_GOAL_OFFSET = 1.55f;
static const float TIMEOUT = 10.65f;
static const float FORWARD_EXE_COOLDOWN = 1.578;

static move_base_msgs::MoveBaseActionGoal navGoal;
static actionlib_msgs::GoalID cancelNavCmd; // leave empty is ok, by default means cancel
static robot_message::voice_cast voiceMsg;

Target target;
Action action;
Location location;
std::string message = "";

nav_msgs::OccupancyGrid map;
bool receivedPrinted = false;
void localMapCb(const nav_msgs::OccupancyGrid msg)
{
    if (!receivedPrinted)
    {
        std::cout << "-----=====================================>>>>.map received\n";
        receivedPrinted = true;
    }
    map = msg;
}
hkust_rgd_msgs::gestures gestures;
void gesturesCb(const hkust_rgd_msgs::gestures msg)
{
    gestures = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgd_control_node");
    gestures.msg = "NONE";

    ros::NodeHandle nh("~");
    ROS_ERROR_STREAM("--> Let's go");
    float lastRxTime = ros::Time::now().toSec();
    float lastForwardRxTime = ros::Time::now().toSec();
    // for publishing nav goal to move_base to navigate
    ros::Publisher navGoalPub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 100);

    // for publishing voice output message to feedback to user
    // ros::Publisher voiceOutputPub = nh.advertise<robot_message::voice_cast>("/voice_cast", 100);

    ros::Publisher navCancelPub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::ServiceClient client = nh.serviceClient<robot_message::SLU>("/slu");
    // robot_message::SLU srv;
    // srv.request.header.seq = 1;

    // ros::Subscriber mapSubscriber = nh.subscribe("/projected_map", 1, localMapCb);
    ros::Subscriber gestureSubscriber = nh.subscribe("/gestures", 1, gesturesCb);

    navGoal.goal.target_pose.header.frame_id = "odom";

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Rate(3).sleep();

        // char key = getch();

        // RECEIVE MESSAGE
        // if (client.call(srv))
        // {
        // }
        // else
        // {
        //     ROS_ERROR("Failed to call");
        //     continue;
        // }
        // std::cout << srv.response.action << std::endl;
        // DEBUG MODE
        // std::cout << "\n"
        //           << key << std::endl;
        // if (key == 'r')
        //     message = "please move right";
        // else if (key == 'l')
        //     message = "please move left";
        // else if (key == 'f')
        //     message = "please move forward";
        // else if (key == 's')
        //     message = "please stop";

        // DECODE MESSAGE
        // mapStr2Target(std::string(srv.response.action).c_str(), &target);
        // mapStr2Action(std::string(srv.response.action).c_str(), &action);
        // mapStr2Location(std::string(srv.response.action).c_str(), &location);
        ROS_WARN_STREAM("--> Current gesture message: " << gestures.msg);
        if (strcmp(gestures.msg.c_str(), "NONE") == 0)
        {
            float dur = ros::Time::now().toSec() - lastRxTime;
            if (dur > TIMEOUT)
            {
                ROS_WARN_STREAM("--> Timeout! Manually stop");
                gestures.msg = "stop";
            }
            else
                continue;
        }
        else{
            lastRxTime = ros::Time::now().toSec();
        }
        mapStr2Action(gestures.msg.c_str(), &action);
        // GET THE CURRENT ROBOT POSITION AND ORIENTATION
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PointStamped goalPntAtBaseFootPrint;
        geometry_msgs::PointStamped goalPntAtMap;
        goalPntAtBaseFootPrint.header.frame_id = "base_footprint";
        goalPntAtBaseFootPrint.header.seq = 1;
        goalPntAtBaseFootPrint.point.x = 1;
        goalPntAtBaseFootPrint.point.y = 0;
        goalPntAtBaseFootPrint.point.z = 0;
        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
                                                        ros::Time(0));

            tfBuffer.transform(goalPntAtBaseFootPrint, goalPntAtMap, "map");

            ROS_INFO("Position of goal in map (x:%f y:%f z:%f)\n",
                     goalPntAtMap.point.x,
                     goalPntAtMap.point.y,
                     goalPntAtMap.point.z);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // OPEN THIS FOR DEBUG
        // std::cout << "current robot position: "
        //           << "x->" << transformStamped.transform.translation.x << " y->" << transformStamped.transform.translation.y << " z->" << transformStamped.transform.translation.z << std::endl;

        // TAKE APPROPRIATE ACTION
        switch (action)
        {
        case Action::ACTION_NONE:
        {
            continue;
        }

        case Action::ACTION_MOVE_FORWARD:
        {
            if ((ros::Time::now().toSec() - lastForwardRxTime) < FORWARD_EXE_COOLDOWN)
            {
                break;
            }

            lastForwardRxTime = ros::Time::now().toSec();
            ROS_WARN_STREAM("--> Forward execution");
            voiceMsg.msg = "going forward, going forward";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion frontVecQ_base_footprint(-NAV_GOAL_OFFSET * 1.5f, 0, 0, 0);
            tf2::Quaternion frontVecQ_odom = rotQ * frontVecQ_base_footprint * rotQInv; // check this ?frame to ?frame

            // set a nav goal to go to the forward for some distance
            navGoal.goal.target_pose.pose.position.x = transformStamped.transform.translation.x + frontVecQ_odom.getX();
            navGoal.goal.target_pose.pose.position.y = transformStamped.transform.translation.y + frontVecQ_odom.getY();
            navGoal.goal.target_pose.pose.position.z = 0;

            navGoal.goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            navGoal.goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            navGoal.goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
            navGoal.goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

            // TODO: check if this goal point overlaps with any obstacles in the current map

            navGoalPub.publish(navGoal);
            break;
        }

        case Action::ACTION_MOVE_LEFT:
        {
            std::cout << "left\n";
            voiceMsg.msg = "going left, going left";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion leftVecQ_base_footprint(0, -NAV_GOAL_OFFSET / 5.0f, 0, 0);
            tf2::Quaternion leftVecQ_odom = rotQ * leftVecQ_base_footprint * rotQInv; // check this ?to?

            // set a nav goal to go to the left for some distance
            navGoal.goal.target_pose.pose.position.x = transformStamped.transform.translation.x + leftVecQ_odom.getX();
            navGoal.goal.target_pose.pose.position.y = transformStamped.transform.translation.y + leftVecQ_odom.getY();
            navGoal.goal.target_pose.pose.position.z = 0;

            navGoal.goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            navGoal.goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            navGoal.goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
            navGoal.goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

            navGoalPub.publish(navGoal);
            break;
        }

        case Action::ACTION_MOVE_RIGHT:
        {
            std::cout << "right \n";
            voiceMsg.msg = "going right, going right";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion rightVecQ_base_footprint(0, NAV_GOAL_OFFSET / 5.0f, 0, 0);
            tf2::Quaternion rightVecQ_odom = rotQ * rightVecQ_base_footprint * rotQInv; // check this ?to?

            // set a nav goal to go to the right for some distance
            navGoal.goal.target_pose.pose.position.x = transformStamped.transform.translation.x + rightVecQ_odom.getX();
            navGoal.goal.target_pose.pose.position.y = transformStamped.transform.translation.y + rightVecQ_odom.getY();
            navGoal.goal.target_pose.pose.position.z = 0;

            navGoal.goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            navGoal.goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            navGoal.goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
            navGoal.goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

            navGoalPub.publish(navGoal);
            break;
        }

        case Action::ACTION_FIND:
        {
            continue;
        }

        case Action::ACTION_STOP:
        {
            std::cout << "stop \n";
            voiceMsg.msg = "stop, stop";

            navCancelPub.publish(cancelNavCmd);
            break;
        }

        default:
        {
            break;
        }
        }

        navGoal.header.seq += 1;
        // voiceMsg.header.frame_id = "both";
        // voiceMsg.prefix = "notice";

        // voiceOutputPub.publish(voiceMsg);
    }
}