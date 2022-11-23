#include <ros/ros.h>
#include <hkust_rgd_msgs/voice_cast.h>
#include <hkust_rgd_msgs/rgd_command.h>
#include <iostream>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <robot_message/VoiceReco.h>
#include <robot_message/voice_cast.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>

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

static const float NAV_GOAL_OFFSET = 2.8f;

static move_base_msgs::MoveBaseActionGoal navGoal;
static actionlib_msgs::GoalID cancelNavCmd; // leave empty is ok, by default means cancel
static robot_message::voice_cast voiceMsg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgd_control_node");

    ros::NodeHandle nh("~");

    // for publishing nav goal to move_base to navigate
    ros::Publisher navGoalPub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 100);

    // for publishing voice output message to feedback to user
    ros::Publisher voiceOutputPub = nh.advertise<robot_message::voice_cast>("/voice_cast", 100);

    ros::Publisher navCancelPub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::ServiceClient client = nh.serviceClient<robot_message::VoiceReco>("/voice_recognition");
    robot_message::VoiceReco srv;
    srv.request.header.seq = 1;

    navGoal.goal.target_pose.header.frame_id = "odom";

    while (ros::ok())
    {

        ros::spinOnce();

        ros::Rate(2).sleep();

        // wait for user input
        getch();

        // call service for voice recognition
        if (client.call(srv))
        {
        }
        else
        {
            ROS_ERROR("Failed to call");
            continue;
        }

        // get the current robot position and orientation
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
                                                        ros::Time(0));
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

        navGoal.header.seq += 1;

        if (std::string(srv.response.cmd).find("go") != std::string::npos)
        {
            std::cout << "go \n";
            voiceMsg.msg = "going forward, going forward";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion frontVecQ_base_footprint(-NAV_GOAL_OFFSET, 0, 0, 0);
            tf2::Quaternion frontVecQ_odom = rotQ * frontVecQ_base_footprint * rotQInv; // check this ?to?

            // set a nav goal to go to the forward for some distance
            navGoal.goal.target_pose.pose.position.x = transformStamped.transform.translation.x + frontVecQ_odom.getX();
            navGoal.goal.target_pose.pose.position.y = transformStamped.transform.translation.y + frontVecQ_odom.getY();
            navGoal.goal.target_pose.pose.position.z = 0;

            navGoal.goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            navGoal.goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            navGoal.goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
            navGoal.goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

            navGoalPub.publish(navGoal);
        }
        else if (std::string(srv.response.cmd).find("right") != std::string::npos)
        {
            std::cout << "right \n";
            voiceMsg.msg = "going right, going right";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion rightVecQ_base_footprint(0, NAV_GOAL_OFFSET, 0, 0);
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
        }
        else if (std::string(srv.response.cmd).find("left") != std::string::npos)
        {
            std::cout << "left\n";
            voiceMsg.msg = "going left, going left";

            tf2::Quaternion rotQ(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Quaternion rotQInv(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, -transformStamped.transform.rotation.w);
            tf2::Quaternion leftVecQ_base_footprint(0, -NAV_GOAL_OFFSET, 0, 0);
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
        }
        else if (std::string(srv.response.cmd).find("stop") != std::string::npos)
        {
            std::cout << "stop \n";
            voiceMsg.msg = "stop, stop";

            navCancelPub.publish(cancelNavCmd);
        }

        voiceMsg.header.frame_id = "both";
        voiceMsg.prefix = "notice";

        ros::Rate(2).sleep();
        voiceOutputPub.publish(voiceMsg);
    }
}