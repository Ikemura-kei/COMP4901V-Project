#include <ros/ros.h>
#include <ford_msgs/Clusters.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

void agentCb(const pedsim_msgs::AgentStates &msg);
ros::Publisher *pub = nullptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_state_2_clusters_node");
    ros::NodeHandle nh;

    ros::Publisher clusterPub = nh.advertise<ford_msgs::Clusters>("/obst_odom", 10);
    ros::Subscriber agentSub = nh.subscribe("/pedsim_simulator/simulated_agents", 5, agentCb);
    pub = &clusterPub;
    ros::spin();
}

static ford_msgs::Clusters clusters;
void agentCb(const pedsim_msgs::AgentStates &msg)
{
    clusters.labels.clear();
    clusters.mean_points.clear();
    clusters.velocities.clear();

    for (int i = 0; i < msg.agent_states.size(); i++)
    {
        clusters.labels.push_back(msg.agent_states.at(i).id);

        geometry_msgs::Point meanPts;
        meanPts.x = msg.agent_states.at(i).pose.position.x;
        meanPts.y = msg.agent_states.at(i).pose.position.y;
        clusters.mean_points.push_back(meanPts);

        geometry_msgs::Vector3 vel;
        vel.x = msg.agent_states.at(i).twist.linear.x;
        vel.y = msg.agent_states.at(i).twist.linear.y;
        clusters.velocities.push_back(vel);
    }

    pub->publish(clusters);
}