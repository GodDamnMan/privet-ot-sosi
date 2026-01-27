import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

class FakeEncoder(Node):
    def __init__(self):
        super().__init__('fake_encoder')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('ticks_per_rev', 2048)
        self.declare_parameter('left_rps', 1.0)  
        self.declare_parameter('right_rps', 1.2) 

        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.left_rps = self.get_parameter('left_rps').value
        self.right_rps = self.get_parameter('right_rps').value

        self.left_ticks = 0
        self.right_ticks = 0

        self.publisher_ = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # msg = Int32MultiArray()
    def timer_callback(self):
        left_rps = self.get_parameter('left_rps').value
        right_rps = self.get_parameter('right_rps').value
        dt = 1.0 / self.rate_hz
        self.left_ticks += int(left_rps * self.ticks_per_rev * dt)
        self.right_ticks += int(right_rps * self.ticks_per_rev * dt)

        msg = Int32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  