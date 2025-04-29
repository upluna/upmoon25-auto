import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import os
import time

# PWM sysfs configuration
PWMCHIP = 2     # pwmchip2 → for Pin 33
CHANNEL = 0     # channel 0

# Servo PWM Specs
PWM_FREQUENCY = 50  # 50Hz (20ms period)
PERIOD_NS = int(1e9 / PWM_FREQUENCY)  # 20,000,000 ns
MIN_DC_PERCENT = 4.95
MAX_DC_PERCENT = 9.5
MAX_RANGE = 100
MIN_RANGE = 0
INIT_RANGE = 0

SLEEP_TIME = 3.0  # seconds
'''
    This node's functionality has been moved to arduino_driver
'''
class SysfsPWM:
    def __init__(self, pwmchip, channel):
        self.pwmchip = pwmchip
        self.channel = channel
        self.base_path = f"/sys/class/pwm/pwmchip{pwmchip}"
        self.pwm_path = f"{self.base_path}/pwm{channel}"

        self.export()
        self.set_period(PERIOD_NS)
        self.set_duty_cycle_percent(INIT_RANGE)
        self.enable()

    def export(self):
        if not os.path.exists(self.pwm_path):
            with open(f"{self.base_path}/export", 'w') as f:
                f.write(str(self.channel))
            time.sleep(0.1)

    def unexport(self):
        if os.path.exists(self.pwm_path):
            with open(f"{self.base_path}/unexport", 'w') as f:
                f.write(str(self.channel))

    def set_period(self, period_ns):
        with open(f"{self.pwm_path}/period", 'w') as f:
            f.write(str(period_ns))

    def set_duty_cycle(self, duty_ns):
        with open(f"{self.pwm_path}/duty_cycle", 'w') as f:
            f.write(str(duty_ns))

    def set_duty_cycle_percent(self, percent):
        if percent < MIN_RANGE or percent > MAX_RANGE:
            percent = INIT_RANGE
        duty_cycle_range = MAX_DC_PERCENT - MIN_DC_PERCENT
        duty_percent = (percent / 100.0) * duty_cycle_range + MIN_DC_PERCENT
        duty_ns = int(PERIOD_NS * duty_percent / 100.0)
        self.set_duty_cycle(duty_ns)

    def enable(self):
        with open(f"{self.pwm_path}/enable", 'w') as f:
            f.write("1")

    def disable(self):
        with open(f"{self.pwm_path}/enable", 'w') as f:
            f.write("0")

    def cleanup(self):
        self.disable()
        self.unexport()

class BucketServos(Node):
    def __init__(self):
        super().__init__('bucket_servos')

        self.pwm = SysfsPWM(PWMCHIP, CHANNEL)
        self.pwm_on = True

        self.timer = self.create_timer(SLEEP_TIME, self.pwm_sleep)

        self.subscription = self.create_subscription(
            Int16,
            'cmd/bucket_pos',
            self.sub_callback,
            10
        )
        self.get_logger().info('Bucket Servos Node Initialized')

    def pwm_sleep(self):
        if self.pwm_on:
            self.pwm.set_duty_cycle_percent(100)  # Sleep with fully "on" duty (no movement)
            self.pwm_on = False

    def sub_callback(self, msg):
        self.timer.reset()
        if not self.pwm_on:
            self.pwm_on = True
        self.pwm.set_duty_cycle_percent(msg.data)

    def destroy_node(self):
        self.pwm.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BucketServos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
