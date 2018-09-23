import math
import nav_msgs.msg
import rospy
import tf.transformations

class Odometry:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.ang = 0

    def integrate(self, dist, ang):
        # convert distance to meter, ang to radians
        ang = math.radians(ang)

        self.x += math.cos(ang) * dist
        self.y += math.sin(ang) * dist

        self.ang += ang

    def to_msg(self):
        # convert to an odometry message

        m = nav_msgs.msg.Odometry()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"

        m.pose.covariance[0]  = 0.02
        m.pose.covariance[7]  = 0.02
        m.pose.covariance[14] = 0.02
        m.pose.covariance[21] = 0.05
        m.pose.covariance[28] = 0.05
        m.pose.covariance[35] = 0.05

        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        quat = tf.transformations.quaternion_about_axis(self.ang, (0, 0, 1))
        m.pose.pose.orientation.x = quat[0]
        m.pose.pose.orientation.y = quat[1]
        m.pose.pose.orientation.z = quat[2]
        m.pose.pose.orientation.w = quat[3]

        return m
