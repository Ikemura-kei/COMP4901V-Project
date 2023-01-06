#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
    class DynamicObstacleSim : public WorldPlugin
    {
    public:
        DynamicObstacleSim() : WorldPlugin()
        {
        }

        void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ROS_INFO_STREAM("DynamicObstacleSim plugin loaded!");
        }
    };

    GZ_REGISTER_WORLD_PLUGIN(DynamicObstacleSim)
}
