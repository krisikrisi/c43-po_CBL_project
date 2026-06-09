#!/usr/bin/env python3

from r2drip2.base import Base, Plot

import json
from pathlib import Path
from datetime import datetime

from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

# Constants:
DIGITAL_FARM_PATH = "digital_farm.json"
OPERATION_LOG_PATH = "operation_log.json"

class FarmManager(Base):
    """
    **Subscriptions**
    - /watering_done

    **Publishers**
    - /water_change

    **Services**
    - /get_state
    - /get_crops

    Attributes
    ----------
    farm_state : array
        The current state of all the farm plots
    operation_logs : array
        All the operations the robot has executed
    watering_done_subscription : Subscription
        The subscription listening for /watering_done
    water_change_publisher : Publisher
        Publishes to /water_change when a cell's moisture changes
    get_state_service : Service
        Returns the current moisture of all cells
    get_crops_service : Service
        Returns the current crop of all cells
    """

    def __init__(self):
        super().__init__('farm_manager')

        # Subscribe to /watering_done (from robot_mover.py)
        self.watering_done_subscription = self.create_subscription(
            Int32,
            '/watering_done',
            self.watering_done_callback,
            10
        )

        self.water_change_publisher = self.create_publisher(String, '/water_change', 10)

        self.get_state_service = self.create_service(Trigger, '/get_state', self.get_state_callback)
        self.get_crops_service = self.create_service(Trigger, '/get_crops', self.get_crops_callback)

        self.load_files()
        self.info("Farm manager node started")

    def load_files(self):
        """
        Loads the state (from the previous time the program ran) from the disk
        If it cannot find the files, it loads appropriate replacement data
        """
        if not self.data_file_exists(DIGITAL_FARM_PATH):
            self.error("Could not find the digital farm file ('/data/" + DIGITAL_FARM_PATH + "')")
            self.farm_state = {
                "cells": {
                    str(i): {
                        "moisture": 50
                    }
                    for i in range(9)
                }
            }
        else:
            self.farm_state = self.read_json(DIGITAL_FARM_PATH)

        if not self.data_file_exists(OPERATION_LOG_PATH):
            self.error("Could not find the operational log file ('/data/" + OPERATION_LOG_PATH + "')")
            self.operation_logs = {
                "logs": []
            }
        else:
            self.operation_logs = self.read_json(OPERATION_LOG_PATH)

    def shutdown(self):
        """
        Makes sure the state of the program is written to the disk
        """
        self.write_json(DIGITAL_FARM_PATH, self.farm_state)
        self.write_json(OPERATION_LOG_PATH, self.operation_logs)

        self.watering_done_subscription.destroy()
        super().destroy()

    def watering_done_callback(self, msg):
        """
        Called when ROS receive a message from  /watering_done

        Parameters
        ----------
        msg : WateringDone
            The plot where watering was done
        """
        plot = Plot(msg.data)

        if not plot.valid():
            self.error(f"Invalid cell number: {plot.get_key()}")
            return

        self.on_plot_watered(plot)

    def on_plot_watered(self, plot):
        """
        Updates the farm state when a plot is watered

        Parameters
        ----------
        plot : Plot
            The plot that was watered
        """
        old_moisture = self.farm_state["cells"][str(plot.get_key())]["moisture"]
        new_moisture = old_moisture + 5
        self.farm_state["cells"][str(plot.get_key())]["moisture"] += 5
        log_entry = {
            "time": datetime.now().isoformat(timespec='seconds'),
            "cell": plot.get_key(),
            "action": "watered"
        }
        self.operation_logs["logs"].append(log_entry)
        self.info(f"Cell {plot.get_key()} moisture: {old_moisture}% -> {new_moisture}%")

        out = String()
        out.data = json.dumps({"cell": plot.get_key(), "moisture": new_moisture})
        self.water_change_publisher.publish(out)

    def get_state_callback(self, request, response):
        """
        Returns the current moisture level of every cell

        Parameters
        ----------
        request : Trigger.Request
            Empty request
        response : Trigger.Response
            response.message is JSON of the form {"0": {"moisture": 55}, ...}
        """
        state_data = {
            cell_id: {"moisture": cell["moisture"]}
            for cell_id, cell in self.farm_state["cells"].items()
        }
        response.success = True
        response.message = json.dumps(state_data)
        return response

    def get_crops_callback(self, request, response):
        """
        Returns the current crop of every cell

        Parameters
        ----------
        request : Trigger.Request
            Empty request
        response : Trigger.Response
            response.message is JSON of the form {"0": {"plant": "corn"}, ...}
        """
        crops_data = {
            cell_id: {"plant": cell.get("plant", "unknown")}
            for cell_id, cell in self.farm_state["cells"].items()
        }
        response.success = True
        response.message = json.dumps(crops_data)
        return response


def main(args=None):
    node = FarmManager()
    node.process()
    node.shutdown()


if __name__ == '__main__':
    main()