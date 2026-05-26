#!/usr/bin/env python3

import json
from pathlib import Path
from datetime import datetime

import rclpy # this will be a node
from rclpy.node import Node
from std_msgs.msg import Int32


class FarmManager(Node):
    def __init__(self):
        super().__init__('farm_manager')

        # this is a subscriber, move_to_cell is a publisher
        self.subscription = self.create_subscription(
            Int32,
            '/watering_done',
            self.watering_done_callback,
            10
        )

        self.data_dir = Path(__file__).parent / "../data" # from the farm manager file go ../data

        self.digital_farm_path = self.data_dir / "digital_farm.json"
        self.operation_log_path = self.data_dir / "operation_log.json"

        self.get_logger().info("Farm manager node started")

    def read_json(self, path):
        with open(path, 'r', encoding='utf-8') as file:
            return json.load(file)

    def write_json(self, path, data):
        with open(path, 'w', encoding='utf-8') as file:
            json.dump(data, file, indent=4) # add 4 spaces basically

    def watering_done_callback(self, msg): # callback is something ros calls when topic is updated 
        cell_number = msg.data # unpack the cell num from the Int32 message we got

        if cell_number < 0 or cell_number > 8:
            self.get_logger().error(f"Invalid cell number: {cell_number}")
            return

        self.update_digital_farm(cell_number)
        self.write_operation_log(cell_number)

        self.get_logger().info(f"Cell {cell_number} was watered")

    def update_digital_farm(self, cell_number):
        digital_farm = self.read_json(self.digital_farm_path)

        cell_key = str(cell_number)

        old_moisture = digital_farm["cells"][cell_key]["moisture"]
        new_moisture = old_moisture + 5

        digital_farm["cells"][cell_key]["moisture"] = new_moisture

        self.write_json(self.digital_farm_path, digital_farm)

        self.get_logger().info(
            f"Cell {cell_number} moisture: {old_moisture}% -> {new_moisture}%"
        )

    def write_operation_log(self, cell_number):
        operation_log = self.read_json(self.operation_log_path)

        log_entry = {
            "time": datetime.now().isoformat(timespec='seconds'),
            "cell": cell_number,
            "action": "watered"
        }

        operation_log["logs"].append(log_entry)

        self.write_json(self.operation_log_path, operation_log)


def main(args=None):
    rclpy.init(args=args)

    node = FarmManager()

    try:
        rclpy.spin(node) # ros pls keep this node alive and keep listening for events
    except KeyboardInterrupt: # in case of ctrl c just pss to next
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()