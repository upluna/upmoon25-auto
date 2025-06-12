import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from std_msgs.msg import Int16

'''
    The purpose of this node is to determine when the bucket chain gets stuck by monitoring the
    bucket alarm GPIO pin. We made this during our competition and didn't really get to test it,
    but it may be worth looking into.

    Publishes:
    /sensor/bucket_alarm - Int16, 0 means the alarm is off, 1 means the alarm is on
'''''
class BucketAlarm(Node):
    def __init__(self):
        super().__init__('bucket_alarm')

        # Set GPIO pin number (pin numbering)
        self.gpio_pin = 32 
        GPIO.setmode(GPIO.BOARD) #research on this GPIO
        GPIO.setup(self.gpio_pin, GPIO.IN)

        # Create a publisher for Boolean messages
        self.publisher = self.create_publisher(Int16, '/sensor/bucket_alarm', 10)
        self.timer = self.create_timer(0.1, self.pub_callback)
        self.get_logger().info('Bucket Alarm Node Initialized')

    def pub_callback(self):
        msg = Int16()
        if (GPIO.input(self.gpio_pin) == 1):
            msg.data = 0    # 0 means the alarm is off
        else:
            msg.data = 1    # 1 means the alarm is on
        
        print(msg.data)
        self.publisher.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BucketAlarm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()