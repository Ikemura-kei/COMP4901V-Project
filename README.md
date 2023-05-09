# Build
Just run `catkin_make` in the workspace. Make sure you meet the installation requirement of ORB-SLAM3 at their official github page first.

# Run ORB-SLAM3 based navigation
After doing `source devel/setup.bash`, you can launch the simulation that uses ORB-SLAM3 by the following command:

```
roslaunch hkust_rgd_bringup orb_slam3_nav_bringup.launch
```

# Run Lidar-Inertia-SLAM based navigation
You need to modify a file. Go to `src/hkust_rgd_navigation/launch/move_base.launch` and comment-out line 16.  

Then you can run the following command to launch the navigation simulation:

```
roslaunch hkust_rgd_bringup lidar_slam_nav_bringup.launch
```

# Run non-SLAM based navigation
Assume you haven't modified anything to this project, you need to perform a few modification to able to run this. First, go to `src/hkust_rgd_navigation/launch/move_base.launch` and comment-out line 10, 11, and 16. Then, in the same file, un-comment line 7.

Then you can run the following to launch the navigation simulation:

```
roslaunch hkust_rgd_bringup baseline_nav.launch
```