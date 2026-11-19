"""
teleop_keyboard: drive the car from your PC's keyboard over Wi-Fi.

MANUAL MODE ONLY. When the RC receiver / autonomy take over, delete this file
and its line in setup.py. Nothing else uses it.

How to use: SSH into the Jetson from your PC, then run
    ros2 run aerohawk_drive teleop_keyboard
(drive_node must be running in another terminal.)

Publishes /cmd_vel 10 times per second, which keeps drive_node's safety stop happy.
If this program closes or Wi-Fi drops, drive_node stops the car on its own.
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
AeroHawk keyboard drive
  w / s  : faster / slower (keep pressing s to reverse)
  a / d  : steer left / right
  x      : straighten steering
  space  : STOP (speed and steering to zero)
  q      : quit (also stops)
"""

SPEED_STEP = 0.1     # m/s per key press
TURN_STEP = 0.2      # rad/s per key press
PUBLISH_HZ = 10.0


def read_key(timeout_s):
    """Return one key press, or '' if nothing was pressed within timeout_s."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    return sys.stdin.read(1) if ready else ''


def main():
    rclpy.init()
    node = Node('teleop_keyboard')
    # drive_node's max_duty is the real power limit; these just cap the command range
    max_speed = node.declare_parameter('max_speed_mps', 1.0).value
    max_turn = node.declare_parameter('max_turn_rad_s', 1.0).value
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.0
    turn = 0.0
    old_terminal = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        tty.setcbreak(sys.stdin.fileno())   # read keys instantly; Ctrl+C still works
        while rclpy.ok():
            key = read_key(1.0 / PUBLISH_HZ).lower()
            if key == 'w':
                speed += SPEED_STEP
            elif key == 's':
                speed -= SPEED_STEP
            elif key == 'a':
                turn += TURN_STEP
            elif key == 'd':
                turn -= TURN_STEP
            elif key == 'x':
                turn = 0.0
            elif key == ' ':
                speed = 0.0
                turn = 0.0
            elif key == 'q':
                break

            speed = round(max(-max_speed, min(max_speed, speed)), 2)
            turn = round(max(-max_turn, min(max_turn, turn)), 2)

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = turn
            publisher.publish(msg)
            print(f'\rspeed {speed:+.1f} m/s   turn {turn:+.1f} rad/s   ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            publisher.publish(Twist())       # all zeros = stop
        except Exception:
            pass                             # ROS already shut down; drive_node's safety stop covers it
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        print('\nstopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
