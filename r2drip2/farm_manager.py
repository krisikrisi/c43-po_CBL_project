#!/usr/bin/env python3

from r2drip2.base import Base, Plot

import json
from datetime import datetime

from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# Constants:
DIGITAL_FARM_PATH = "digital_farm.json"
OPERATION_LOG_PATH = "operation_log.json"

REFRESH_SECONDS = 5


class FarmManager(Base):
    """
    **Subscriptions**
    - /watering_done

    Attributes
    ----------
    farm_state : array
        The current state of all the farm plots
    operation_logs : array
        All the operations the robot has executed
    watering_done_subscription : Subscription
        The subscription listening for /watering_done
    """

    def __init__(self):
        super().__init__('farm_manager')

        self.watering_done_subscription = self.create_subscription(
            Int32,
            '/watering_done',
            self.watering_done_callback,
            10
        )

        self.load_files()

        self.weather_state = {}

        self.service_client = self.create_client(Trigger, srv_name='/get_weather')

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.info("Service /get_weather not available, waiting...")

        self.weather_future = None

        self.refresh_timer = self.create_timer(
            timer_period_sec=REFRESH_SECONDS,
            callback=self.update_timer_callback
        )

        self.update_timer_callback()

        self.info("Farm manager node started")

    def load_files(self):
        """
        Loads the state from disk.
        If files are missing, creates default in-memory data.
        """
        if not self.data_file_exists(DIGITAL_FARM_PATH):
            self.error("Could not find the digital farm file ('/data/" + DIGITAL_FARM_PATH + "')")
            self.farm_state = {
                "last_updated": datetime.now().isoformat(timespec="seconds"),
                "cells": {
                    str(i): {
                        "moisture": 50,
                        "plant": "wheat",
                        "growth_stage": "growing",
                        "planted_date": "2026-05-01",
                        "estimated_harvest_date": "2026-09-01",
                        "status": "needs_monitoring"
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
        Makes sure the state of the program is written to disk.
        """
        self.write_json(DIGITAL_FARM_PATH, self.farm_state)
        self.write_json(OPERATION_LOG_PATH, self.operation_logs)

        self.watering_done_subscription.destroy()
        super().destroy()

    def update_timer_callback(self):
        """
        Called every REFRESH_SECONDS
        Sends request to the weather service
        Updates farm moisture using the latest weather state
        """

        if self.weather_future is not None and not self.weather_future.done():
            self.info("Weather request is pending")
            return

        request = Trigger.Request()
        self.weather_future = self.service_client.call_async(request)
        self.weather_future.add_done_callback(self.weather_response_callback)

    def weather_response_callback(self, future):
        """
        Called when weather service responds
        Updates self.weather_state
        """

        try:
            response = future.result()

            if not response.success:
                self.error("Weather service returned failure")
                self.weather_future = None
                return

            self.weather_state = json.loads(response.message)
            self.info(f"Updated weather state: {self.weather_state}")

            moisture_change = self.calculate_weather_moisture_change()

            for cell_key in self.farm_state["cells"]:
                plot = Plot(int(cell_key))
                self.change_cell_moisture(plot, moisture_change)

            self.farm_state["last_updated"] = datetime.now().isoformat(timespec="seconds")
            self.write_json(DIGITAL_FARM_PATH, self.farm_state)

            self.info(f"Farm moisture changed by {moisture_change:.2f} because of the weather")

        except Exception as e:
            self.error(f"Failed to get weather state: {e}")

        self.weather_future = None

    def calculate_weather_moisture_change(self):
        """
        Calculates how much the soils moisture should change.

        Positive value - soil gets wetter
        Negative value - soil dries
        """

        temperature = self.weather_state["temperature"]
        humidity = self.weather_state["humidity"]
        raining = self.weather_state["raining"]
        water_mm_per_day = self.weather_state["water_mm_per_day"]

        if raining:
            return 0.20 + water_mm_per_day * 0.05

        drying = -0.10

        if temperature > 20:
            drying -= (temperature - 20) * 0.01

        if humidity < 50:
            drying += (humidity - 50) * 0.005

        return drying

    def watering_done_callback(self, msg):
        """
        Called when ROS receives a message from /watering_done.
        """
        plot = Plot(msg.data)

        if not plot.valid():
            self.error(f"Invalid cell number: {plot.get_key()}")
            return

        self.on_plot_watered(plot, 5)

    def on_plot_watered(self, plot, watering_amount):
        """
        Updates the farm state when a plot is watered.
        watering_amount is the amount of water added by robot. 
        It is initially chosen by decision making system
        """

        old_moisture, new_moisture = self.change_cell_moisture(
            plot,
            watering_amount
        )

        self.farm_state["last_updated"] = datetime.now().isoformat(timespec="seconds")

        log_entry = {
            "time": datetime.now().isoformat(timespec='seconds'),
            "cell": plot.get_key(),
            "action": "watering_completed",
            "water_amount_liters": watering_amount,
            "message": "Robot completed watering"
        }

        self.operation_logs["logs"].append(log_entry)

        self.write_json(DIGITAL_FARM_PATH, self.farm_state)
        self.write_json(OPERATION_LOG_PATH, self.operation_logs)

        self.info(f"Cell {plot.get_key()} moisture: {old_moisture}% -> {new_moisture}%")

    def change_cell_moisture(self, plot, amount):
        """
        Changes moisture of one cell
        
        Parameters
        ----------
        plot : Plot
          The plot whose moisture will be changed
        amount : float
          The amount of moisture to add or remove
          If positive, it increases moisture, and negative value decreases moisture

        Returns
        -------
        tuple
          The old moisture value and the new moisture value
        """

        cell_key = str(plot.get_key())
        old_moisture = self.farm_state["cells"][cell_key]["moisture"]
        new_moisture = old_moisture + amount

        new_moisture = max(0, min(100, new_moisture))
        new_moisture = round(new_moisture, 2)

        self.farm_state["cells"][cell_key]["moisture"] = new_moisture

        return old_moisture, new_moisture


def main(args=None):
    node = FarmManager()
    node.process()
    node.shutdown()


if __name__ == '__main__':
    main()