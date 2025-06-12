import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from std_msgs.msg import Int16

'''

    This node operates the conveyor belt on the robot.

    Subscriptions:
    /cmd/conveyor - Int16, 1 to turn on the conveyor, 0 to turn off
'''''
class Conveyor(Node):
    def __init__(self):
        super().__init__('conveyor')

        # Set GPIO pin number (pin numbering)
        self.gpio_pin = 36 
        GPIO.setmode(GPIO.BOARD) #research on this GPIO
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        GPIO.output(self.gpio_pin, GPIO.HIGH)

        # Create subscriber to receive Boolean messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/conveyor',
            self.sub_callback,
            10
        )
        self.get_logger().info('Conveyor Node Initialized')

    def sub_callback(self, msg):
        if (msg.data == 1):
            GPIO.output(self.gpio_pin, GPIO.LOW)
        else:
            GPIO.output(self.gpio_pin, GPIO.HIGH)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Conveyor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()