#!/usr/bin/env python3

# This file contains a Base node and the classes that can be used for communication between nodes

import math
from pathlib import Path # File path
import time
import json
import os
import traceback

import rclpy
from rclpy.node import Node

class Base(Node):
    """
    A base node every node in this project can use.
    This node provides usefull functions for use in the other nodes

    See the Example class on how to use this class

    Attributes
    ----------
    data_dir : string
        The path to the data directory
    up : bool
        If the node hasn't been shutdown yet
    """

    def __init__(self, name):
        """
        Starts the node and rcply
        """
        rclpy.init()
        super().__init__(name)
        self.data_dir = Path(__file__).parent.parent.parent.parent / "data"
        self.up = True
    
    def destroy(self):
        """
        Cleans up the node and shutsdown rcply
        """

        self.up = False
        super().destroy_node()
        rclpy.shutdown()
    
    def ok(self):
        """
        Returns wether the communication is still up (and the node)

        Returns
        -------
        bool
            Wether the communication is still up (and the node)
        """
        return rclpy.ok() and self.up
    
    def process_once(self, timeout=0.1):
        """
        Awaits until a message appears (this node is subscribed to), or until the timeout has passed

        Parameters
        ----------
        timeout : float
            The timeout in seconds
        """
        rclpy.spin_once(self, timeout_sec=timeout)

    def process(self):
        """
        Keep awaiting messages until we get ctrl+c'ed
        This will take control of the process and the rest of the code cannot be executed anymore until this function returns
        """
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.error("Something went wrong while keeping the node alive (during call to process) '" + str(e) + '". See the following stacktrace: ')
            self.error(traceback.format_exc())
            

    def sleep(self, seconds):
        """
        Make the process sleep for x seconds
        Parameters
        ----------
        seconds : float
            The amount to sleep in seconds
        """
        time.sleep(seconds)

    def debug(self, message):
        """
        Logs to the terminal (level **debug**)

        Parameters
        ----------
        message : str
            The message to log
        """
        self.get_logger().debug(message)

    def info(self, message):
        """
        Logs to the terminal (level **info**)

        Parameters
        ----------
        message : str
            The message to log
        """
        self.get_logger().info(message)
    
    def warning(self, message):
        """
        Logs to the terminal (level **warning**)

        Parameters
        ----------
        message : str
            The message to log
        """
        self.get_logger().warning(message)

    def error(self, message):
        """
        Logs to the terminal (level **error**)

        Parameters
        ----------
        message : str
            The message to log
        """
        self.get_logger().error(message)

    def data_file_exists(self, path):
        """
        Checks if a JSON file exists

        Parameters
        ----------
        path : str
            The path (relative to the data directory) to read from
        
        Returns
        -------
        bool
            If the file exists
        """

        data_path = self.data_dir / path
        return data_path.exists()

    def read_json(self, path, replacement_data=None):
        """
        Safely reads a JSON file

        Parameters
        ----------
        path : str
            The path (relative to the data directory) to read from
        replacement_data : Any
            The data to return, if no file at the path exists, or if the file isn't valid JSON
        
        Returns
        -------
        Any
            The object contained in the JSON file, or replacement_data
        """

        data_path = self.data_dir / path
        if not data_path.exists():
            self.info('Could not find the file "' + str(data_path) + '"')
            return replacement_data

        try:
            with open(data_path, 'r', encoding='utf-8') as file:
                return json.load(file)
        except Exception as e:
            self.warning('Error while loading the file "' + str(data_path) + '": "' + str(e) + '"')
            return replacement_data

    def write_json(self, path, data):
        """
        Safely writes to a JSON file (if it fails, it will write so to the terminal and do nothing else)

        Parameters
        ----------
        path : str
            The path (relative to the data directory) to read from
        data : Any
            The data to write to the file
        """
        data_path = self.data_dir / path
        try:
            with open(data_path, 'w', encoding='utf-8') as file:
                json.dump(data, file, indent=4) # add 4 spaces basically
        except Exception as e:
            self.warning('Error while writing to the file "' + str(data_path) + '": "' + str(e) + '"')

class Vec2:
    """
    A vector of height 2

    Attributes
    ----------
    x : float
    y : float
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def set_x(self, x):
        self.x = x
    def set_y(self, y):
        self.y = y
    
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    
    def rotated(self, degree, around=None):
        """
        Receive the rotated vector around the paramater 'around'

        Parameters
        ----------
        degree : float
            The degree in radians to rotate this vector
        around : Vec2
            The point to rotate around (defaults to (0,0))
        """
        # Default argument of 0,0
        if around == None:
            around = Vec2(0,0)
        # Math:
        # 1. First center vector around 'around'
        # 2. Then rotate the vector around (0,0)
        # 3. Add 'around' back
        rotated = Vec2(self.x, self.y)
        rotated -= around

        cos = math.cos(degree)
        sin = math.sin(degree)

        rotated.x = around.get_x() + rotated.x * cos - rotated.y * sin
        rotated.y = around.get_y() + rotated.x * sin + rotated.y * cos
        return rotated

    def rotate(self, degree, around=None):
        """
        Rotate the vector around the paramater 'around'

        Parameters
        ----------
        degree : float
            The degree in radians to rotate this vector
        around : Vec2
            The point to rotate around (defaults to (0,0))
        """
        self = rotated(degree, around)
    
    def length(self):
        """
        Returns the length of the vector

        Returns
        -------
        float
            The length of the vector
        """
        return math.sqrt(self.x*self.x + self.y*self.y)
    
    def angle(self):
        """
        Returns the angle ?from the x axis?

        Returns
        -------
        float
            The angle this vector has compared to the x axis
        """
        return math.atan2(self.y, self.x)

    def __add__(self, other):
        """
        Overloaded + operator

        Parameters
        ----------
        other : Vec2
            The vector to add to this
        """
        return Vec2(self.get_x() + other.get_x(), self.get_y() + other.get_y())
    def __sub__(self, other):
        """
        Overloaded - operator
        
        Parameters
        ----------
        other : Vec2
            The vector to remove from this
        """
        return Vec2(self.get_x() - other.get_x(), self.get_y() - other.get_y())
    def __eq__(self, other):
        """
        Overloaded == operator
        
        Parameters
        ----------
        other : Vec2
            The vector to compare to
        """
        return self.get_x() == other.get_x() and self.get_y() == other.get_y()
    def __ne__(self, other):
        """
        Overloaded != operator
        
        Parameters
        ----------
        other : Vec2
            The vector to compare to
        """
        return not(self == other)
    def __iadd__(self, other):
        """
        Overloaded += operator
        
        Parameters
        ----------
        other : Vec2
            The vector to add to this
        """
        self.x += other.get_x()
        self.y += other.get_y()
        return self
    def __isub__(self, other):
        """
        Overloaded -= operator
        
        Parameters
        ----------
        other : Vec2
            The vector to remove from this
        """
        self.x -= other.get_x()
        self.y -= other.get_y()
        return self
    def __imul__(self, other):
        """
        Overloaded -= operator
        
        Parameters
        ----------
        other : float
            Factor to multiply with
        """
        self.x *= other
        self.y *= other
        return self
    def __itruediv__(self, other):
        """
        Overloaded /= operator
        
        Parameters
        ----------
        other : float
            Factor to divide with
        """
        self.x /= other
        self.y /= other
        return self
    def __ifloordiv__(self, other):
        """
        Overloaded /= operator
        
        Parameters
        ----------
        other : float
            Factor to divide with
        """
        self.x //= other
        self.y //= other
        return self

class Position:
    """
    A position of the robot

    Attributes
    ----------
    pos : Vec2
    yaw : float
    """
    def __init__(self, x, y, yaw):
        self.pos = Vec2(x, y)
        self.yaw = yaw

    def set_x(self, x):
        self.pos.x = x
    def set_y(self, y):
        self.pos.y = y
    def set_pos(self, pos):
        self.pos = pos
    def set_yaw(self, yaw):
        self.yaw = yaw
    
    def get_x(self):
        return self.pos.x
    def get_y(self):
        return self.pos.y
    def get_pos(self):
        return self.pos
    def get_yaw(self):
        return self.yaw

# Constants:
CELL_SIZE = 0.5
# 0 1 2
# 3 4 5
# 6 7 8
CELL_POS = [
    Vec2(CELL_SIZE, CELL_SIZE),
    Vec2(CELL_SIZE, 0.0),
    Vec2(CELL_SIZE, -CELL_SIZE),
    Vec2(0.0, CELL_SIZE),
    Vec2(0.0, 0.0),
    Vec2(0.0, -CELL_SIZE),
    Vec2(-CELL_SIZE, CELL_SIZE),
    Vec2(-CELL_SIZE, 0.0),
    Vec2(-CELL_SIZE, -CELL_SIZE)
]
class Plot:
    """
    The identifier of a cell

    Attributes
    ----------
    key : int
        The plot key
    """

    def __init__(self, key):
        self.key = key

    def valid(self):
        """
        Returns if the plot is valid
        """
        return self.key >= 0 and self.key <= 8
    
    def get_key(self):
        return self.key
    
    def get_x(self):
        return CELL_POS[self.key][0]

    def get_y(self):
        return CELL_POS[self.key][1]

    def get_pos(self):
        return CELL_POS[self.key]



class Example(Base):
    """
    A class showing how to use the Base node
    """
    def __init__(self):
        super().__init__('example_node')

        self.debug("Test log message")
        self.info("Test info message")
        self.warning("Test warn message")
        self.error("Test error message")

def main(args=None):
    node = Example()
    node.destroy()

if __name__ == '__main__':
    main()