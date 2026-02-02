import rclpy
import sys
import math
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qx, qy, qz, qw


class Wheel_Odom_Boadcaster(Node):
    def __init__(self):
        super().__init__('wheel_odom_broadcaster_static_tf2')
        self.declare_parameter("wheel_base", 0.30)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")

        self.r = float(self.get_parameter("wheel_radius").value)
        self.b = float(self.get_parameter("wheel_base").value)
        self.left_name = str(self.get_parameter("left_joint_name").value)
        self.right_name = str(self.get_parameter("right_joint_name").value)


        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None


        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        # self.broadcaster = StaticTransformBroadcaster(self)
        # self.make_transforms(transformation)
        #
        
        # def timer_callback(self):
        #         pass
    def on_joint_state(self, msg: JointState):
        try:
            i_l = msg.name.index(self.left_name)
            i_r = msg.name.index(self.right_name)
        except ValueError:
            # Joint names not found in message
            return

        if len(msg.position) <= max(i_l, i_r):
            return

        left = float(msg.position[i_l])
        right = float(msg.position[i_r])

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9  
        
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_time = t
            return

        dt = t - self.prev_time
        if dt <= 0.0:
            return

        # Wheel angle deltas (rad)
        dtheta_l = left - self.prev_left
        dtheta_r = right - self.prev_right

        self.prev_left = left
        self.prev_right = right
        self.prev_time = t

        # Differential drive kinematics
        ds_l = self.r * dtheta_l
        ds_r = self.r * dtheta_r

        ds = 0.5 * (ds_r + ds_l)
        dyaw = (ds_r - ds_l) / self.b

        # Midpoint integration
        mid_yaw = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw += dyaw

        # Velocities
        v = ds / dt
        w = dyaw / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        tfm = TransformStamped()
        tfm.header.stamp = now.to_msg()
        tfm.header.frame_id = "odom"
        tfm.child_frame_id = "base_link"

        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tfm)


def main(args=None):
    rclpy.init(args=args)
    node = Wheel_Odom_Boadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()