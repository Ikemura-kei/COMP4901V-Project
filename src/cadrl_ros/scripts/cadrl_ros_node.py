#!/usr/bin/env python
import rospy
from hkust_rgd_msgs.msg import dynamic_obstacles, agent_actions
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, Twist, Vector3
import copy
import sys

# internal modules
import rospkg
from cadrl_ros import agent
from cadrl_ros import util
from cadrl_ros import network

class CadrlRosNode():
    def __init__(self, neural_network, actions, vehicle_data, vehicle_name):
        self.node_name = rospy.get_name()
        self.vehicle_name = vehicle_name
        self.vehicle_data = vehicle_data
        self.nn = neural_network
        self.actions = actions

        # agent status
        self.pose = PoseStamped()
        self.vel = Vector3()
        self.psi = 0.0
        self.ped_traj_vec = []
        self.other_agents_state = []  # list of 'Agent' objects
        # list of 'Agent' objects of previous time-stamp
        self.prev_other_agents_state = []
        self.feasible_actions = agent_actions()

        # for publishers
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        self.goal.pose.position.x = vehicle_data['goal'][0]  # x
        self.goal.pose.position.y = vehicle_data['goal'][1]  # y
        self.desired_position = PoseStamped()
        self.desired_action = np.zeros((2,))

        # handle obstacles close to vehicle's front
        self.stop_moving_flag = False
        self.new_subgoal_received = False
        self.new_global_goal_received = False
        self.prev_clusters = dynamic_obstacles()
        self.current_clusters = dynamic_obstacles()
        self.num_poses = 0
        self.num_actions_computed = 0.0

        # publishers
        self.pub_twist = rospy.Publisher('/velocity_controller/cmd_vel', Twist, queue_size=1)

        # subscribers
        self.sub_clusters = rospy.Subscriber(
            '/dynamic_obstacles', dynamic_obstacles, self.cbDynamicObstacles)
        self.sub_pose = rospy.Subscriber('~pose', PoseStamped, self.cbPose)
        self.sub_vel = rospy.Subscriber('~velocity', Vector3, self.cbVel)
        self.sub_global_goal = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.cbGlobalGoal)

        # control timer
        self.control_timer = rospy.Timer(
            rospy.Duration(0.01), self.cbControl)  # 100Hz control
        self.nn_timer = rospy.Timer(rospy.Duration(
            0.1), self.cbComputeActionGA3C)  # 10Hz update

    def cbDynamicObstacles(self, msg):
        '''
        in this callback we update 'self.other_agents' variable

        msg: hkust_rgd_msgs.dynamic_obstacles
        '''
        num_clusters = len(msg.pose)
        other_agents = []

        for i in range(num_clusters):
            # should be parameters / variables
            radius = 0.55
            inflation_factor = 1.52

            radius *= inflation_factor
            x = msg.pose[i].position.x
            y = msg.pose[i].position.y
            vx = msg.twist[i].linear.x
            vy = msg.twist[i].linear.y
            # arctan2(vy/vx), is the heading w.r.t. the x-axis
            heading_angle = np.arctan2(vy, vx)
            # the speed of the obstacle
            pref_speed = np.linalg.norm(np.array([vx, vy]))
            goal_x = x + 5.0
            goal_y = y + 5.0  # set a hypothetical goal of the obstacle

            if pref_speed < 0.2:  # why do we have this?
                pref_speed = 0
                vx = 0
                vy = 0

            # other_agents.append(agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)) # do we nee to track obstacles?
            # try this for now, 0 is reserved for the self
            other_agents.append(agent.Agent(
                x, y, goal_x, goal_y, radius, pref_speed, heading_angle, i+1))
            
            # rospy.loginfo("received obstacle [%d]" % (i + 1))

        self.other_agents_state = other_agents

    def cbPose(self, msg):
        '''
        in this callback we update 'self.num_poses', 'self.pose', and 'self.psi' variable

        msg: geometry_msgs.PoseStamped
        '''
        self.num_poses += 1
        q = msg.pose.orientation
        # quaternion to yaw angle
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y),
                              1-2*(q.y*q.y+q.z*q.z))  # bounded by [-pi, pi]
        self.pose = msg
        # rospy.loginfo("received position: [x: %.3f], [y: %.3f]" % (self.pose.pose.position.x, self.pose.pose.position.y))

    def cbVel(self, msg):
        '''
        in this callback we update 'self.vel' variable

        msg: geometry_msgs.Vector3
        '''
        self.vel = msg
        # rospy.loginfo("received velocity: [x: %.3f], [y: %.3f], [z: %.3f]" % (msg.x, msg.y, msg.z))

    def cbGlobalGoal(self, msg):
        '''
        in this callback we update 'self.new_global_goal_received', 'self.goal', and 'self.new_subgoal_received' variable

        msg: geometry_msgs.PoseStamped
        '''
        self.new_global_goal_received = True
        self.global_goal = msg

        self.goal.pose.position.x = msg.pose.position.x
        self.goal.pose.position.y = msg.pose.position.y
        self.goal.header = msg.header

        self.new_subgoal_received = True
        rospy.loginfo("received goal: [x: %.3f], [y: %.3f], [z: %.3f]" % (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z))

    def cbControl(self, event):
        '''
        in this callback we update 'self.pub_twist' variables

        event: rospy.timer.TimerEvent
        '''
        # stop the movement if the timestamp of the goal is invalid or when we want to stop
        if self.goal.header.stamp == rospy.Time(0) or self.stop_moving_flag and not self.new_global_goal_received:
            self.stop_moving()

            return
        else:
            # desired_action = [x_velocity, target_yaw_angle]
            desired_yaw = self.desired_action[1]
            yaw_error = desired_yaw - self.psi

            # map the error in the range of [-pi, pi]
            if abs(yaw_error) > np.pi:
                yaw_error -= np.sign(yaw_error) * 2 * np.pi

            # these should be parameters
            kp = 1.65

            # P control
            vw = kp * yaw_error
            # the larger the yaw error, the smaller the x velocity, to prioritize in-place rotation first
            vx = min(self.desired_action[0],
                     self.find_vmax(yaw_error))
            print("desired velocity: [%.5f], desired heading: [%.5f]" % (self.desired_action[0], self.desired_action[1]))

            twist = Twist()
            twist.angular.z = vw
            twist.linear.x = vx

            self.pub_twist.publish(twist)

            return

    def cbComputeActionGA3C(self, event):
        '''
        in this callback we update 'self.stop_moving_flag', 'self.desired_action', 'self.desired_position' variables

        event: rospy.timer.TimerEvent
        '''
        # construct the agent_state
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        vx = self.vel.x
        vy = self.vel.y

        radius = self.vehicle_data['radius']
        heading_angle = self.psi
        pref_speed = self.vehicle_data['pref_speed']
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y

        # in case the current speed is larger than desired speed, shrink the current speed
        v = np.linalg.norm(np.array([vx, vy]))
        if v > pref_speed:
            vx = vx * pref_speed / v
            vy = vy * pref_speed / v

        host_agent = agent.Agent(x, y, goal_x, goal_y,
                                 radius, pref_speed, heading_angle, 0)
        host_agent.vel_global_frame = np.array([vx, vy])

        other_agents_state = copy.deepcopy(self.other_agents_state)
        obs = host_agent.observe(other_agents_state)[1:]
        obs = np.expand_dims(obs, axis=0)

        # apply neural network to predict actions
        predictions = self.nn.predict_p(obs)[0]
        # raw_action = [vx, delta_yaw]
        raw_action = copy.deepcopy(self.actions[np.argmax(predictions)])
        # process the raw action a bit
        action = np.array([pref_speed * raw_action[0],
                          util.wrap(raw_action[1] + self.psi)])
        rospy.loginfo("raw delta heading: [%.5f], psi: [%.5f]" % (raw_action[1], self.psi))
        kp_v = 0.5
        kp_r = 1
        # slow down if close to goal
        if host_agent.dist_to_goal < 2.0:
            pref_speed = max(
                min(kp_v * (host_agent.dist_to_goal-0.1), pref_speed), 0.0)
            action[0] = min(raw_action[0], pref_speed)
            turn_amount = max(
                min(kp_r * (host_agent.dist_to_goal-0.1), 1.0), 0.0) * raw_action[1]
            action[1] = util.wrap(turn_amount + self.psi)
        # stop if reaching into a tolerable range of the goal
        if host_agent.dist_to_goal < 0.3:
            self.stop_moving_flag = True
        # otherwise normal operation
        else:
            self.stop_moving_flag = False

        self.update_action(action)

    def stop_moving(self):
        '''
        this function stops the agent by publishing a null geometry_msgs.Twist message
        '''
        twist = Twist()
        self.pub_twist.publish(twist)

    def update_action(self, action):
        '''
        this function updates 'self.desired_action' and 'self.desired_position'

        action: [vx, yaw]
        '''
        self.desired_action = action
        self.desired_position.pose.position.x = self.pose.pose.position.x + \
            action[0] * np.cos(action[1])
        self.desired_position.pose.position.y = self.pose.pose.position.y + \
            action[0] * np.sin(action[1])

    def find_vmax(self, heading_diff):
        '''
        this function calculates the appropriate vx as a function of the yaw_error, the larger the 
        yaw error, the smaller the x velocity, to prioritize in-place rotation first
        '''
        if abs(heading_diff) < np.pi / 12:
            return self.vehicle_data['pref_speed']
        return self.vehicle_data['pref_speed'] * 0.45

    def on_shutdown(self):
        '''
        this function performs the actions when the program needs to be shutdown
        '''
        rospy.loginfo("[%s] Shutting down." % (self.node_name))
        self.stop_moving()
        rospy.loginfo("Stopped %s's velocity." % (self.vehicle_data))

def run():
    print('running cadrl_ros_node.py')

    plt.rcParams.update({'font.size': 18})
    rospack = rospkg.RosPack()

    a = network.Actions()
    actions = a.actions
    num_actions = a.num_actions
    nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
    nn.simple_load(rospack.get_path('cadrl_ros') + '/checkpoints/network_01900000')

    rospy.init_node('cadrl_ros_node', anonymous=False)
    vehicle_name = 'sentry_robot'
    pref_speed = rospy.get_param("~sentry_speed")
    vehicle_data = {'goal': np.zeros((2,)), 'radius': 0.5, 'pref_speed': pref_speed, 'kw': 10.0, 'kp': 1.0, 'name': 'sentry_robot'}

    node_obj = CadrlRosNode(nn, actions, vehicle_data, vehicle_name)
    rospy.on_shutdown(node_obj.on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    print("python version:", sys.version)
    run()
