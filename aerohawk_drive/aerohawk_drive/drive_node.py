"""
drive_node: turns /cmd_vel into steering + motor commands.

Listens to /cmd_vel (geometry_msgs/Twist):
    linear.x   forward speed in m/s (negative = reverse)
    angular.z  turn rate in rad/s (positive = turn left)

Anything can publish /cmd_vel: teleop_keyboard now, the RC receiver or Nav2 later.
This node doesn't care who, so it never has to change when autonomy is added.

Safety: if no /cmd_vel arrives for cmd_timeout_s (Wi-Fi drop, teleop closed),
the motor stops and the steering centers.

Settings come from aerohawk_bringup/config/drive_params.yaml.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from aerohawk_drive.hardware import build_hardware


def twist_to_commands(linear_x, angular_z, max_speed_mps, wheelbase_m,
                      max_steering_rad, min_steer_speed_mps):
    """Convert a Twist into (motor speed -1..1, steering -1..1) using car-style steering."""
    speed = max(-1.0, min(1.0, linear_x / max_speed_mps))
    if angular_z == 0.0:
        return speed, 0.0

    # A car can't turn in place. When (almost) stopped, pretend we're moving at
    # min_steer_speed so the wheels still turn when you steer while parked.
    v = linear_x
    if abs(v) < min_steer_speed_mps:
        v = min_steer_speed_mps if v >= 0 else -min_steer_speed_mps

    angle = math.atan(wheelbase_m * angular_z / v)          # positive = wheels left
    steering = max(-1.0, min(1.0, angle / max_steering_rad))
    return speed, -steering                                  # SteeringServo: negative = left


class DriveNode(Node):
    def __init__(self):
        # Load every setting straight from drive_params.yaml - no duplicate list here
        super().__init__('drive_node', automatically_declare_parameters_from_overrides=True)
        settings = {name: p.value for name, p in self.get_parameters_by_prefix('').items()}

        self.pca, self.steering, self.motor = build_hardware(settings)

        try:
            self.max_speed_mps = settings['max_speed_mps']
            self.wheelbase_m = settings['wheelbase_m']
            self.max_steering_rad = math.radians(settings['max_steering_angle_deg'])
            self.min_steer_speed_mps = settings['min_steer_speed_mps']
            self.cmd_timeout_s = settings['cmd_timeout_s']
            update_rate_hz = settings['update_rate_hz']
        except KeyError as missing:
            raise RuntimeError(f"setting {missing} is missing from drive_params.yaml") from None

        self.last_cmd_time = None
        self.timed_out = True          # start stopped until the first command arrives
        self.steering.center()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / update_rate_hz, self.on_timer)
        self.get_logger().info('drive_node ready, waiting for /cmd_vel')

    def on_cmd_vel(self, msg):
        speed, steering = twist_to_commands(
            msg.linear.x, msg.angular.z, self.max_speed_mps, self.wheelbase_m,
            self.max_steering_rad, self.min_steer_speed_mps)
        self.motor.set_speed(speed)
        self.steering.set_steering(steering)
        self.last_cmd_time = self.get_clock().now()
        if self.timed_out:
            self.timed_out = False
            self.get_logger().info('receiving /cmd_vel')

    def on_timer(self):
        if self.timed_out:
            return
        age_s = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age_s > self.cmd_timeout_s:
            self.timed_out = True
            self.motor.stop()
            self.steering.center()
            self.get_logger().warn(f'no /cmd_vel for {age_s:.1f}s - motor stopped')
            return
        self.motor.update()            # ramp toward the commanded speed

    def shutdown(self):
        self.motor.stop()
        self.steering.center()
        self.pca.close()               # every channel off


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()