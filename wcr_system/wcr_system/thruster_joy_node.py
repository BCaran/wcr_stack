import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float32MultiArray

# SDL2 GameController mapping (ROS 2 Humble `joy`)
BTN_L1 = 9
BTN_R1 = 10
BTN_DPAD_UP = 11
BTN_DPAD_DOWN = 12


class ThrusterJoyNode(Node):

    def __init__(self):
        super().__init__('thruster_joy_node')

        # Limits and step sizes as parameters, so they can be changed from the launch file
        self.thr_max = self.declare_parameter('thr_max', 0.6).value
        self.edf_max = self.declare_parameter('edf_max', 0.6).value
        self.thr_step = self.declare_parameter('thr_step', 0.01).value
        self.edf_step = self.declare_parameter('edf_step', 0.01).value

        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.thr_publisher = self.create_publisher(Float32MultiArray, '/wcr/thrusters/duty_cycle', 10)
        self.edf_publisher = self.create_publisher(Float64, '/wcr/edf/duty_cycle', 10)

        self.thr = 0.0
        self.edf = 0.0

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(x, hi))

    def joy_callback(self, msg: Joy):
        b = msg.buttons
        if len(b) <= BTN_DPAD_DOWN:
            self.get_logger().warn(
                f'Unexpected Joy layout: {len(b)} buttons', throttle_duration_sec=2.0)
            return

        if b[BTN_DPAD_UP]:
            self.thr += self.thr_step
        if b[BTN_DPAD_DOWN]:
            self.thr -= self.thr_step
        if b[BTN_R1]:
            self.edf += self.edf_step
        if b[BTN_L1]:
            self.edf -= self.edf_step

        self.thr = round(self.clamp(self.thr, 0.0, self.thr_max), 2)
        self.edf = round(self.clamp(self.edf, 0.0, self.edf_max), 2)

        self.thr_publisher.publish(Float32MultiArray(data=[self.thr] * 4))
        self.edf_publisher.publish(Float64(data=self.edf))
        self.get_logger().info(
            f'Thrusters: {self.thr:.2f}  EDF: {self.edf:.2f}', throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()