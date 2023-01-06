import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf

class MsgTransformNode():
    def __init__(self):
        self.pub_pose = rospy.Publisher('/cadrl_ros_node/pose', PoseStamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/cadrl_ros_node/velocity', Vector3, queue_size=1)
        
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, callback=self.cbOdom)
        self.pose = PoseStamped()
        self.velocity = Vector3()
        self.listener = tf.TransformListener()
        
        return
        
    def cbOdom(self, msg):
        '''
        this callback receives odometry data and republish it to pose, velocity
        
        msg: nav_msgs/Odometry
        '''
        self.pose.pose.position.x = msg.pose.pose.position.x
        self.pose.pose.position.y = msg.pose.pose.position.y
        self.pose.pose.orientation = msg.pose.pose.orientation
        
        vel_body = Vector3Stamped()
        vel_body.header.frame_id = msg.child_frame_id
        vel_body.vector.x = msg.twist.twist.linear.x
        vel_body.vector.y = msg.twist.twist.linear.y
        
        try:
            vel_global = self.listener.transformVector3("odom", vel_body)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf exception encountered")
            return
        
        self.velocity.x = vel_global.vector.x
        self.velocity.y = vel_global.vector.y
        
        self.pub_pose.publish(self.pose)
        self.pub_vel.publish(self.velocity)
        
        return
    
def run():
    print('running msg_transform_node.py')

    rospy.init_node('msg_transform_node', anonymous=False)
    
    node_obj = MsgTransformNode()

    rospy.spin()
    
if __name__ == '__main__':
    run()
        
        