#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped # type of message for /cmd_vel
from nav_msgs.msg import Odometry # type of message for /odom


class MoveToCell(Node):
    def __init__(self):          # creating new node "move to cell"
        super().__init__('move_to_cell')

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10) # publisher to send movement commands

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # listen to /odom - position of robot. ros will call odom_callback when new messages come

        self.cell_size = 0.5

        # 0 1 2
        # 3 4 5
        # 6 7 8
        self.cells = [
            (self.cell_size, self.cell_size),
            (self.cell_size, 0.0),
            (self.cell_size, -self.cell_size),
            (0.0, self.cell_size),
            (0.0, 0.0),
            (0.0, -self.cell_size),
            (-self.cell_size, self.cell_size),
            (-self.cell_size, 0.0),
            (-self.cell_size, -self.cell_size)
        ]

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None

        self.odom_received = False
        self.is_moving = False

    def odom_callback(self, msg):  # updating coordinates (from odom). it takes x, y, orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation # in ros operations run in quaternions (4dimensional)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp) #yaw - the angle of rotation of the robot on the plane
        self.odom_received = True

    def make_twist(self, linear_x=0.0, angular_z=0.0): # creating message of twisttimestamp
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def wait_for_odom(self): # waiting for the first odom so that the robot doesnt move until its coordinates are unknown
        if not self.odom_received:
            self.get_logger().info("Waiting for /odom...")

        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)         # ros checking incoming messages. if /odom appeared, call callback

    def set_origin_if_needed(self): # save start position so that cell 4 is like (0, 0).
        if self.origin_x is None:
            self.origin_x = self.current_x
            self.origin_y = self.current_y
            self.origin_yaw = self.current_yaw

            self.get_logger().info(
                f"Origin saved: x={self.origin_x:.2f}, "
                f"y={self.origin_y:.2f}, yaw={self.origin_yaw:.2f}"
            )

    def get_target_position(self, cell_number): # coordinate of the destination cell
        local_x, local_y = self.cells[cell_number]

        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)

        target_x = self.origin_x + local_x * cos_yaw - local_y * sin_yaw
        target_y = self.origin_y + local_x * sin_yaw + local_y * cos_yaw

        return target_x, target_y

    def normalize_angle(self, angle): # from -pi to pi only (to not turn 350* when can be -10*)
        return math.atan2(math.sin(angle), math.cos(angle))


    def go_to_cell(self, cell_number): # main function
        if cell_number < 0 or cell_number > 8:
            self.get_logger().error(f"Wrong cell number: {cell_number}")
            return False

        self.wait_for_odom()        # get odom
        self.set_origin_if_needed()          

        target_x, target_y = self.get_target_position(cell_number)  # coords of cell

        self.get_logger().info(
            f"Going to cell {cell_number}: x={target_x:.2f}, y={target_y:.2f}"
        )

        self.is_moving = True

        while rclpy.ok() and self.is_moving:
            rclpy.spin_once(self, timeout_sec=0.01)

            dx = target_x - self.current_x
            dy = target_y - self.current_y

            distance = math.sqrt(dx * dx + dy * dy)

            if distance < 0.07:
                self.stop()
                self.get_logger().info(f"Reached cell {cell_number}")
                self.is_moving = False
                return True

            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            if abs(angle_diff) < 0.05: # if looking almost at destination, move
                linear_x = min(0.08, distance)
                angular_z = 0.0
            else: # else rotate
                linear_x = 0.0
                angular_z = 0.25 * angle_diff
                angular_z = max(min(angular_z, 0.30), -0.30) # max speed of turn

            msg = self.make_twist(linear_x, angular_z)
            self.publisher.publish(msg) # send message to /cmd_vel

            self.get_logger().info(
                f"dist={distance:.2f}, angle_diff={angle_diff:.2f}, "
                f"linear={linear_x:.2f}, angular={angular_z:.2f}"
            )
            time.sleep(0.1)

        self.stop()
        return False

    def stop(self):
        msg = self.make_twist(0.0, 0.0)
        self.publisher.publish(msg)


def print_menu():
    print("\n" + "=" * 50)
    print("TurtleBot3 Burger - Move to Cell")
    print("=" * 50)
    print("0    1    2")
    print("3    4    5")
    print("6    7    8")
    print("=" * 50)
    print("Cell 4 = robot start position")
    print("Distance between cells = 0.5 m")
    print("Enter cell number 0-8")
    print("Enter -1 to exit")
    print("=" * 50)


def main(args=None):
    rclpy.init(args=args)      # starts ros for python

    node = MoveToCell() # creating node

    node.wait_for_odom() # waiting for pos and saving start position
    node.set_origin_if_needed()

    print_menu()

    while rclpy.ok(): # while ros is working
        try:
            user_input = input("\nEnter cell number: ")
            cell = int(user_input)

            if cell == -1:
                print("Exiting...")
                break

            if 0 <= cell <= 8:
                node.go_to_cell(cell)
            else:
                print("Invalid. Use 0-8 or -1.")

        except ValueError:
            print("Enter a number.")

        except KeyboardInterrupt:
            print("\nExiting...")
            break

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()