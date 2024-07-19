#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

terminal_msg = """
Turtlebot3 Velocity Control
------------------------------------------------------
Linear: vx (unit: m/sec)
Angular: wz (unit: rad/sec)
------------------------------------------------------
"""

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        input_vx, input_wz = self.get_key()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist = Twist()
        self.twist.linear.x = input_vx  # Forward speed
        self.twist.angular.z = input_wz  # No rotation


    def timer_callback(self):
        self.publisher_.publish(self.twist)

    def get_key(self):
        # Print terminal message and get inputs
        print(terminal_msg)
        input_vx = float(input("Input Linear: "))
        input_wz = float(input("Input Angular: "))
      
        # settings = termios.tcgetattr(sys.stdin)
        # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_vx, input_wz



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()