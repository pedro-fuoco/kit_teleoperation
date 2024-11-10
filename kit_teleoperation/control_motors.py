import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pynput import keyboard

FORWARD_DUTY = 100.0
BACKWARD_DUTY = -100.0
TURN_DUTY = 70.0
STOP_DUTY = 0.0

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        self.left_pub = self.create_publisher(Float32, '/kit/wheels/left/duty_cycle', 10)
        self.right_pub = self.create_publisher(Float32, '/kit/wheels/right/duty_cycle', 10)
        
        self.left_duty = STOP_DUTY
        self.right_duty = STOP_DUTY
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def publish_duty_cycle(self):
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = self.left_duty
        right_msg.data = self.right_duty
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.get_logger().info(f"Published Left Duty: {self.left_duty}, Right Duty: {self.right_duty}")

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.left_duty = FORWARD_DUTY
                self.right_duty = FORWARD_DUTY
            elif key.char == 's':
                self.left_duty = BACKWARD_DUTY
                self.right_duty = BACKWARD_DUTY
            elif key.char == 'a':
                self.left_duty = -TURN_DUTY
                self.right_duty = TURN_DUTY
            elif key.char == 'd':
                self.left_duty = TURN_DUTY
                self.right_duty = -TURN_DUTY
            self.publish_duty_cycle()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in ['w', 'a', 's', 'd']:
                self.left_duty = STOP_DUTY
                self.right_duty = STOP_DUTY
                self.publish_duty_cycle()
        except AttributeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
