#!/usr/bin/env python3

from r2drip2.base import Base, Plot, Position, CELL_POS

import math
import copy

from sensor_msgs.msg import BatteryState
try:
    from turtlebot3_msgs.srv import Sound
    TURTLEBOT_SOUND = True
except ImportError:
    TURTLEBOT_SOUND = False


BATTERY_ALERT_PERCENT = 0.2


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
        self.water_cell_subscription = self.create_subscription(Int32, '/water_cell', self.water_cell_callback, 10)  # listen to /water_cell - cell the decision system wants watered
        self.battery_subscription = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.current_pos = Position(0,0,0)
        self.origin = None

        if TURTLEBOT_SOUND:
            self.info("Sound is enabled!")
            self.sound_client = self.create_client(Sound, '/sound')

        self.odom_received = False
        self.next_cell = None  # set by water_cell_callback, handled in main loop
        self.publish_vel(0.0, 0.0)
        self.battery_percentage = 1

    def odom_callback(self, msg):  # updating coordinates (from odom). it takes x, y, orientation
        self.current_pos.set_x(msg.pose.pose.position.x)
        self.current_pos.set_y(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation # in ros operations run in quaternions (4dimensional)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) #yaw - the angle of rotation of the robot on the plane

        self.current_pos.set_yaw(yaw)
        self.odom_received = True

    def water_cell_callback(self, msg):  # decision system tells us which cell to water next
        self.next_cell = msg.data
    
    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage

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

    def play_sound(self, sound_id, done_callback=None):
        """
        Sends a sound request to the /sound service
        OFF           = 0
        ON            = 1
        LOW_BATTERY   = 2
        ERROR         = 3
        BUTTON1       = 4
        BUTTON2       = 5
        """
        if not TURTLEBOT_SOUND:
            return
        request = Sound.Request()
        request.value = sound_id
        call = self.sound_client.call_async(request)
        if done_callback != None:
            call.add_done_callback(done_callback)

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
        if self.battery_percentage <= BATTERY_ALERT_PERCENT:
            self.play_sound(3)
        self.play_sound(1)
        msg = Int32()
        msg.data = plot.get_key()
        self.done_publisher.publish(msg)
        self.info(f"Published watering done for cell {plot.get_key()}")
        self.sleep(2)


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


def main(args=None):
    node = RobotMover() # creating node

    node.wait_for_odom() # waiting for position and saving start position
    node.set_origin_if_needed()

    try:
        while node.ok(): # while ros is working
            node.process_once() # returns as soon as a /water_cell message arrives

            if node.next_cell is not None:
                cell = node.next_cell
                node.next_cell = None
                node.go_to_cell(cell)  # go_to_cell validates the cell number itself
    except KeyboardInterrupt:
        print("\nExiting...")

    node.stop()


if __name__ == '__main__':
    main()
