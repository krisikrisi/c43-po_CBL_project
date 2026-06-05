#!/usr/bin/env python3

from r2drip2.base import Base, Plot

import json
from pathlib import Path
from datetime import datetime

from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# Constants:
DIGITAL_FARM_PATH = "digital_farm.json"
OPERATION_LOG_PATH = "operation_log.json"

class FarmManager(Base):
    """
    Manages the digital twin of the farm.

    Subscribed
    /watering_done (Int32) - robot finished watering a cell

    Publishes
    /water_change (Int32) - cell ID whose moisture just changed
    /crop_change  (Int32) - cell ID whose crop just changed

    Services
    /get_state - returns current moisture of all cells as JSON
    /get_crops - returns current crop of all cells as JSON

    Notes
    JSON files are only read at startup and written at shutdown.
    All runtime communication happens via ROS topics and services.
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

        # Publishers
        self.water_change_publisher = self.create_publisher(Int32, '/water_change', 10)
        self.crop_change_publisher = self.create_publisher(Int32, '/crop_change', 10)

        # Services
        self.get_state_service = self.create_service(Trigger, '/get_state', self.get_state_callback)
        self.get_crops_service = self.create_service(Trigger, '/get_crops', self.get_crops_callback)

        self.load_files()
        self.info("Farm manager node started")

    def load_files(self):
        """
        Loads state from disk into memory. These files are only read here;
        the farm_state dict is the single source of truth while the node runs.
        """
        if not self.data_file_exists(DIGITAL_FARM_PATH):
            self.error("Could not find the digital farm file ('/data/" + DIGITAL_FARM_PATH + "')")
            self.farm_state = {
                "cells": {
                    str(i): {
                        "moisture": 50,
                        "plant": "unknown"
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
        Writes current state to disk and destroys the node.
        """
        self.write_json(DIGITAL_FARM_PATH, self.farm_state)
        self.write_json(OPERATION_LOG_PATH, self.operation_logs)

        self.watering_done_subscription.destroy()
        super().destroy()

    # Services 

    def get_state_callback(self, request, response):
        """
        /get_state: returns the current moisture level of every cell.

        Response message (JSON)
        {
          "cells": {
            "0": {"moisture": 55},
            ...
          }
        }
        """
        state_data = {
            "cells": {
                cell_id: {"moisture": cell["moisture"]}
                for cell_id, cell in self.farm_state["cells"].items()
            }
        }
        response.success = True
        response.message = json.dumps(state_data)
        return response

    def get_crops_callback(self, request, response):
        """
        /get_crops: returns the current crop of every cell.

        Response message (JSON)
        {
          "cells": {
            "0": {"plant": "corn"},
            ...
          }
        }
        """
        crops_data = {
            "cells": {
                cell_id: {"plant": cell.get("plant", "unknown")}
                for cell_id, cell in self.farm_state["cells"].items()
            }
        }
        response.success = True
        response.message = json.dumps(crops_data)
        return response

    # Subscription callbacks

    def watering_done_callback(self, msg):
        """
        Called when /watering_done is received from robot_mover.

        Parameters
        msg : Int32
            Cell ID that was just watered.
        """
        plot = Plot(msg.data)

        if not plot.valid():
            self.error(f"Invalid cell number: {plot.get_key()}")
            return

        self.on_plot_watered(plot)

    # Internal helpers

    def on_plot_watered(self, plot):
        """
        Updates the moisture for a cell and publishes /water_change.

        Parameters
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

        # Notify other nodes (e.g. decision_system) that moisture changed
        out = Int32()
        out.data = plot.get_key()
        self.water_change_publisher.publish(out)

    def on_crop_changed(self, plot):
        """
        Call this whenever a cell's crop is changed to notify other nodes.

        Parameters
        plot : Plot
            The plot whose crop was changed.
        """
        out = Int32()
        out.data = plot.get_key()
        self.crop_change_publisher.publish(out)


def main(args=None):
    node = FarmManager()
    node.process()
    node.shutdown()


if __name__ == '__main__':
    main()
