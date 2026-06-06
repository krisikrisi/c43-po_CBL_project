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
    Keeps track of the farm state (moisture levels, crops per cell).

    Loads state from disk on startup and saves it back on shutdown.
    While running, all communication happens through ROS — other nodes
    use the services to read data and subscribe to topics to get updates.

    Attributes
    ----------
    farm_state : dict
        The current in-memory state of all farm cells (moisture, plant, etc.)
    operation_logs : dict
        Log of all watering operations since startup.
    watering_done_subscription : Subscription
        Listens for /watering_done from the robot mover.
    water_change_publisher : Publisher
        Publishes to /water_change when a cell's moisture changes.
    crop_change_publisher : Publisher
        Publishes to /crop_change when a cell's crop changes.
    get_state_service : Service
        Returns the current moisture of all cells.
    get_crops_service : Service
        Returns the current crop of all cells.
    """

    def __init__(self):
        super().__init__('farm_manager')

        # Subscribe to /watering_done (from robot_mover.py)
        self.watering_done_subscription = self.create_subscription(
            Int32, '/watering_done', self.watering_done_callback, 10
        )

        # Publishers
        self.water_change_publisher = self.create_publisher(String, '/water_change', 10)
        self.crop_change_publisher = self.create_publisher(String, '/crop_change', 10)

        # Services
        self.get_state_service = self.create_service(Trigger, '/get_state', self.get_state_callback)
        self.get_crops_service = self.create_service(Trigger, '/get_crops', self.get_crops_callback)

        self.load_files()
        self.info("Farm manager node started")

    def load_files(self):
        """
        Loads the farm state and operation log from disk into memory.

        If either file is missing, a safe default is used instead.
        After this the files are not touched again until shutdown.
        """
        if not self.data_file_exists(DIGITAL_FARM_PATH):
            self.error("Could not find the digital farm file ('/data/" + DIGITAL_FARM_PATH + "')")
            self.farm_state = {
                "cells": {
                    str(i): {"moisture": 50, "plant": "unknown"}
                    for i in range(9)
                }
            }
        else:
            self.farm_state = self.read_json(DIGITAL_FARM_PATH)

        if not self.data_file_exists(OPERATION_LOG_PATH):
            self.error("Could not find the operational log file ('/data/" + OPERATION_LOG_PATH + "')")
            self.operation_logs = {"logs": []}
        else:
            self.operation_logs = self.read_json(OPERATION_LOG_PATH)

    def shutdown(self):
        """
        Saves the current farm state and operation log to disk, then destroys the node.
        """
        self.write_json(DIGITAL_FARM_PATH, self.farm_state)
        self.write_json(OPERATION_LOG_PATH, self.operation_logs)
        self.watering_done_subscription.destroy()
        super().destroy()

    # --- Services ---

    def get_state_callback(self, request, response):
        """
        Handles /get_state service calls.

        Returns the current moisture level of every cell.

        Returns
        -------
        Trigger.Response
            response.message contains JSON of the form:
            {"0": {"moisture": 55}, "1": {"moisture": 50}, ...}
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
        Handles /get_crops service calls.

        Returns the current crop of every cell.

        Returns
        -------
        Trigger.Response
            response.message contains JSON of the form:
            {"0": {"plant": "corn"}, "1": {"plant": "wheat"}, ...}
        """
        crops_data = {
            cell_id: {"plant": cell.get("plant", "unknown")}
            for cell_id, cell in self.farm_state["cells"].items()
        }
        response.success = True
        response.message = json.dumps(crops_data)
        return response

    # --- Subscription callbacks ---

    def watering_done_callback(self, msg):
        """
        Called when /watering_done is received from the robot mover.

        Parameters
        ----------
        msg : Int32
            Cell ID that was just watered.
        """
        plot = Plot(msg.data)

        if not plot.valid():
            self.error(f"Invalid cell number: {plot.get_key()}")
            return

        self.on_plot_watered(plot)

    # --- Internal helpers ---

    def on_plot_watered(self, plot):
        """
        Updates the moisture for a cell, logs it, and publishes /water_change.

        The /water_change message includes the new moisture value so other
        nodes don't need to call /get_state just to find out what changed.

        Parameters
        ----------
        plot : Plot
            The plot that was watered.
        """
        cell_key = str(plot.get_key())
        old_moisture = self.farm_state["cells"][cell_key]["moisture"]
        new_moisture = old_moisture + 5
        self.farm_state["cells"][cell_key]["moisture"] = new_moisture

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

    def on_crop_changed(self, plot, new_plant):
        """
        Publishes /crop_change when a cell's crop is updated.

        Parameters
        ----------
        plot : Plot
            The plot whose crop was changed.
        new_plant : str
            The name of the new crop.
        """
        out = String()
        out.data = json.dumps({"cell": plot.get_key(), "plant": new_plant})
        self.crop_change_publisher.publish(out)


def main(args=None):
    node = FarmManager()
    node.process()
    node.shutdown()


if __name__ == '__main__':
    main()
