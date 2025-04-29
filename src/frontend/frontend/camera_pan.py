import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from std_msgs.msg import Int16

# Define the PWM pin
PWM_PIN = 15

# Servo PWM Specs
PWM_FREQUENCY = 50  # 50Hz (20ms period)
MIN_DC = 2.5
MAX_DC = 12.5
MAX_RANGE = 300
MIN_RANGE = 0
INIT_RANGE = 0

SLEEP_TIME = 10.0

class CameraPan(Node):
    def __init__(self):
        super().__init__('camera_pan')

        # Set GPIO pin number (BCM numbering)
        try:
            GPIO.setmode(GPIO.BOARD)
        except Exception:
            print('GPIO failure')
        try:
            GPIO.setup(PWM_PIN, GPIO.OUT, initial = GPIO.LOW)
        except Exception:
            print('Pin setup failure')

        self.pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY)
        self.pwm.start(self.convertRangeToDutyCycle(INIT_RANGE))
        self.pwm_on = True

        self.timer = self.create_timer(SLEEP_TIME, self.pwm_sleep)

        # Create subscriber to receive position messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/camera_height',
            self.sub_callback,
            10
        )
        self.get_logger().info('Camera Height Node Initialized')

    def pwm_sleep(self):
        #print('PWM sleeping')
        if self.pwm_on:
            self.pwm.ChangeDutyCycle(100)
            self.pwm_on = False

    def sub_callback(self, msg):
        self.timer.reset()
        if not self.pwm_on:
            #print('PWM awake')
            self.pwm_on = True

        self.pwm.ChangeDutyCycle(self.convertRangeToDutyCycle(msg.data))

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

    def convertRangeToDutyCycle(self, angle):
        if (angle < MIN_RANGE or angle > MAX_RANGE):
            print('Servo set out of bounds')
            angle = INIT_RANGE
        dc = (angle * (MAX_DC - MIN_DC) / MAX_RANGE) + MIN_DC
        #print('Setting servo to %f' % (dc))
        return dc

def main(args=None):
    rclpy.init(args=args)
    node = CameraPan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
