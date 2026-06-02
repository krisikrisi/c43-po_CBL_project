#!/usr/bin/env python3

from r2drip2.base import Base, Plot, Position, CELL_POS

import math
import copy

from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped # type of message for /cmd_vel
from nav_msgs.msg import Odometry # type of message for /odom

class RobotMover(Base):
    """
    **Subscriptions**
    - /odom

    **Publishers**
    - /watering_done
    - /cmv_vel

    Attributes
    ----------
    cells : array
        The current state of all the farm plots
    vel_publisher : Publisher
        The publisher to /cmd_vel
    done_publisher : Publisher
        The publisher to /watering_done
    odom_subscription : Subscription
        The subscription listening for /odom
    current_pos : Position
        The current position of the robot
    origin : Position
        The origin (starting position) of the robot
    """

    def __init__(self):
        super().__init__('robot_mover')

        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10) # publisher to send movement commands
        self.done_publisher = self.create_publisher(Int32, '/watering_done', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # listen to /odom - position of robot. ros will call odom_callback when new messages come

        self.current_pos = Position(0,0,0)
        self.origin = None

        self.odom_received = False
        self.publish_vel(0.0, 0.0)

    def odom_callback(self, msg):  # updating coordinates (from odom). it takes x, y, orientation
        self.current_pos.set_x(msg.pose.pose.position.x)
        self.current_pos.set_y(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation # in ros operations run in quaternions (4dimensional)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) #yaw - the angle of rotation of the robot on the plane

        self.current_pos.set_yaw(yaw)
        self.odom_received = True

    def publish_vel(self, linear=0.0, angular=0.0):
        """
        Publishes a command to the /cmd_vel

        Parameters
        ----------
        linear : float
            The forward velocity to give the bot
        angular : float
            The angular momentum to give the bot
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.vel_publisher.publish(msg)

    def wait_for_odom(self):
        """
        Makes sure we have received positional data (so we know where the robot is)
        """
        if not self.odom_received:
            self.info("Waiting for /odom...")

        while self.ok() and not self.odom_received:
            self.process_once(0.1) # Await a message

    def set_origin_if_needed(self): # save start position so that cell 4 is like (0, 0).
        if self.origin is None:
            self.origin = copy.deepcopy(self.current_pos)

            self.info(
                f"Origin saved: x={self.origin.get_x():.2f}, "
                f"y={self.origin.get_y():.2f}, yaw={self.origin.get_yaw():.2f}"
            )

    def get_target_position(self, plot):
        """
        Returns the coordinates of a cell in the robots coordinates system

        Parameters
        ----------
        plot : Plot
            The plot we want the bots coordinates from
        
        Returns
        -------
        Vec2
            The plots position in the bots coordinates system
        """
        local_pos = plot.get_pos()
        # Transform the local coordinates to the bots coordinates:
        return self.origin.get_pos() + local_pos.rotated(self.origin.get_yaw())

    def normalize_angle(self, angle):
        """
        Returns the smallest angle change possible (so 350 degrees becomes -10 degrees)

        Parameters
        ----------
        angle : float
            The angle to transform
        
        Returns
        -------
        float
            The normalized angle
        """
        return math.atan2(math.sin(angle), math.cos(angle))


    def publish_watering_done(self, plot):
        """
        Publishes to the /watering_done message
        
        Parameters
        ----------
        plot : Plot
            The plot that was watered
        """
        msg = Int32()
        msg.data = plot.get_key()
        self.done_publisher.publish(msg)
        self.info(f"Published watering done for cell {plot.get_key()}")


    def go_to_cell(self, cell_number): # main function
        plot = Plot(cell_number)
        if not plot.valid():
            self.error(f"Illegal cell number: {cell_number}")
            return

        self.wait_for_odom()        # get odom coords
        self.set_origin_if_needed()          

        target = self.get_target_position(plot)  # coords of cell

        self.info(
            f"Going to cell {plot.get_key()}: x={target.get_x():.2f}, y={target.get_y():.2f}"
        )

        while self.ok():
            self.process_once(0.01)

            delta = target - self.current_pos.pos
            distance = delta.length()

            if distance < 0.07:
                self.publish_vel(0.0, 0.0)
                self.info(f"Reached cell {plot.get_key()}")
                self.publish_watering_done(plot)
                return

            target_angle = delta.angle()
            angle_diff = self.normalize_angle(target_angle - self.current_pos.get_yaw()) 

            if abs(angle_diff) > 0.08: # If the angle is far from needed, stop going forward and rotate
                linear_x = 0.0
                angular_z = 0.30 * angle_diff
                angular_z = max(min(angular_z, 0.25), -0.25)
            else: # Else move and turn simultaneously
                linear_x = min(0.08, distance)
                angular_z = 0.30 * angle_diff
                angular_z = max(min(angular_z, 0.15), -0.15)

            msg = self.publish_vel(linear_x, angular_z)

            self.info(
                f"dist={distance:.2f}, angle_diff={angle_diff:.2f}, "
                f"linear={linear_x:.2f}, angular={angular_z:.2f}"
            )

            # Sleep, to prevent spamming the robot with an extreme amount of velocity commands
            self.sleep(0.1)
        self.stop()

    def stop(self):
        self.info("Shutting the node down")
        self.publish_vel(0.0, 0.0)
        super().destroy()


def print_menu():
    print("\n" + "=" * 50)
    print("Choose the cell to move to (0-8)")
    print("=" * 50)
    print("0    1    2")
    print("3    4    5")
    print("6    7    8")
    print("=" * 50)
    print("Cell 4 = robot start (center) position")
    print("Distance between cells = 0.5 m")
    print("Enter -1 to exit")
    print("=" * 50)


def main(args=None):
    node = RobotMover() # creating node

    node.wait_for_odom() # waiting for position and saving start position
    node.set_origin_if_needed()

    print_menu()

    while node.ok(): # while ros is working
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


if __name__ == '__main__':
    main()
