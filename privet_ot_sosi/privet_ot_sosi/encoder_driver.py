import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import JointState

    

class EncoderDriver(Node):
    def __init__(self, ticks_per_rev:int = 2048, pub_rate:float = 1):
        super().__init__('encoder_driver')
        self.ticks_per_rev = ticks_per_rev

        self.subscription_ = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.listener_callback,
            10)
        self.ticks:list[int] = []

        self.publisher_ = self.create_publisher(
            JointState, 
            '/joint_states', 
            10)
        self.names:list[str] = ['left_wheel_joint', 'right_wheel_joint']
        self.poses:list[float] = [0., 0.]
        self.vels:list[float] = [0., 0.]
        self.timer = self.create_timer(pub_rate, self.timer_callback)


    def listener_callback(self, msg) -> None:
        # self.get_logger().info(f'I heard: {msg[0]}, {msg[1]}')
        self.ticks = msg.data


    def timer_callback(self):
        msg = JointState()
        msg.name = self.names
        msg.position = [i/self.ticks_per_rev for i in self.ticks]
        msg.velocity = self.vels

        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing {self.names} joint state')



def main(args=None):
    rclpy.init(args=args)

    encoder_driver_node = EncoderDriver()

    rclpy.spin(encoder_driver_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()