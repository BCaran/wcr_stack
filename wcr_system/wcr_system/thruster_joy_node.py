import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64, Int8


class TrhusterJoyNode(Node):

    def __init__(self):
        super().__init__('thruster_joy_node')
        self.joy_subscribtion = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_subscribtion 

        self.thr_publihser = self.create_publisher(Int8, '/wcr/thruster_pwm', 10)
        self.edf_publisher = self.create_publisher(Float64, '/wcr/edf/dutty_cycle', 10)

        self.thruster_pwm = Int8()
        self.edf_duty = Float64()
        
        self.thruster_pwm.data = 0
        self.edf_duty.data = 0.0

    def joy_callback(self, msg):
        if (msg.axes[7] == 1.0):
            self.thruster_pwm.data += 1
            if (self.thruster_pwm.data > 100):
                self.thruster_pwm.data = 100
        if (msg.axes[7] == -1.0):
            self.thruster_pwm.data -= 1
            if(self.thruster_pwm.data < 0):
                self.thruster_pwm.data = 0
        if (msg.buttons[5] == 1):
            self.edf_duty.data += 0.01
            if (self.edf_duty.data > 0.75):
                self.edf_duty.data = 0.75
        if (msg.buttons[4] == 1):
            self.edf_duty.data -= 0.01
            if (self.edf_duty.data < 0.0):
                self.edf_duty.data = 0.0
        self.thr_publihser.publish(self.thruster_pwm)
        self.edf_publisher.publish(self.edf_duty)
        print('Thruster PWM: ', self.thruster_pwm)
        print('EDF Duty: ', self.edf_duty)


def main(args=None):
    rclpy.init(args=args)

    thruster_joy_node = TrhusterJoyNode()

    rclpy.spin(thruster_joy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thruster_joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()