from r2drip2.base import Base
import rclpy
import json
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, String


# File names (relative to the data/ folder; Base knows where that is)
PLANT_DB_PATH = "plant_database.json"
CONFIG_PATH = "system_config.json"

# Tunable defaults
DEFAULT_WATER_LITERS = 2.5  # amount to water per irrigation action


class DecisionMaker(Base):
    """
    Decides which cell to water next and tells the robot to go there.

    Every time a /water_change arrives it runs a decision cycle. It commits
    to the cell furthest below its minimum and keeps watering that same cell
    until it reaches its minimum, only then moving on to the next-worst cell.
    This stops the robot from bouncing back and forth between cells that have
    similar moisture deficits. If enough rain is expected, watering is skipped.

    Attributes
    ----------
    config : dict
        Contents of system_config.json (thresholds and settings).
    plant_info : dict
        Lookup table from plant name to plant data from plant_database.json.
    weather_client : Client
        Service client for /get_weather.
    state_client : Client
        Service client for /get_state (farm_manager).
    crops_client : Client
        Service client for /get_crops (farm_manager).
    water_cell_publisher : Publisher
        Publishes the cell ID the robot should water next.
    should_decide : bool
        Set to True when a new decision cycle should run.
    current_cell : int or None
        The cell currently being watered. Kept until it reaches its minimum
        moisture, then cleared so the next-worst cell can be chosen.
    """

    def __init__(self):
        super().__init__('decision_maker')

        # Load config and plant data once at startup — these don't change while running
        self.config = self.read_json(CONFIG_PATH)
        plants_db = self.read_json(PLANT_DB_PATH)
        self.plant_info = {
            p["name"].lower(): p for p in plants_db["plants"]
        }

        # Service clients
        self.weather_client = self.create_client(Trigger, '/get_weather')
        self.state_client = self.create_client(Trigger, '/get_state')
        self.crops_client = self.create_client(Trigger, '/get_crops')

        # Publisher
        self.water_cell_publisher = self.create_publisher(Int32, '/water_cell', 10)

        # Run a decision cycle once at startup, then again on every /water_change
        self.should_decide = True
        self.current_cell = None  # cell we're currently finishing before moving on
        self.create_subscription(String, '/water_change', self.water_change_callback, 10)

        self.info("Decision maker node started!")

    def water_change_callback(self, msg):
        """
        Called when the farm manager reports a moisture change.

        Just sets a flag — the actual decision runs in the main loop,
        not here. See the comment in main() for why.

        Parameters
        ----------
        msg : String
            JSON with the cell ID and new moisture value.
        """
        self.should_decide = True

    def _call_trigger(self, client):
        """
        Calls a Trigger service and returns the response as a dict.

        Parameters
        ----------
        client : Client
            The service client to call.

        Returns
        -------
        dict or None
            The parsed JSON response, or None if the service is unreachable.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.warning("A required service is not available, skipping")
            return None

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return json.loads(response.message)

    def get_weather(self):
        """
        Calls /get_weather.

        Returns
        -------
        dict or None
            Weather data, or None if the service is unreachable.
        """
        return self._call_trigger(self.weather_client)

    def get_state(self):
        """
        Calls /get_state to get the current moisture of all cells.

        Returns
        -------
        dict or None
            Moisture data per cell, or None if unreachable.
        """
        return self._call_trigger(self.state_client)

    def get_crops(self):
        """
        Calls /get_crops to get the current crop of all cells.

        Returns
        -------
        dict or None
            Crop data per cell, or None if unreachable.
        """
        return self._call_trigger(self.crops_client)

    def decide(self):
        """
        Runs one decision cycle if should_decide is set.

        Keeps watering the current cell until it reaches its minimum moisture,
        then picks the cell furthest below its minimum and commits to that one.
        Does nothing if all cells are fine or if enough rain is expected.
        """
        if not self.should_decide:
            return
        self.should_decide = False

        if self.significant_rain():
            self.info("Significant rain expected; skipping watering")
            return

        state = self.get_state()
        crops = self.get_crops()

        if state is None or crops is None:
            self.warning("Could not reach farm_manager services; skipping decision")
            return

        # Finish the cell we already started on before moving to another one.
        if self.current_cell is not None and self.cell_still_needs_water(self.current_cell, state, crops):
            self.publish_water_cell(self.current_cell)
            self.info(f"Continuing to water cell {self.current_cell}")
            return

        # Current cell is satisfied (or we don't have one) — pick the next-worst cell.
        most_deficient_cell = None
        greatest_deficit = 0

        for cell_id, cell_state in state.items():
            moisture = cell_state["moisture"]
            crop = crops[cell_id]["plant"].lower()
            plant = self.plant_info.get(crop)

            if plant is None:
                self.warning(f"No plant data for '{crop}' (cell {cell_id}), skipping")
                continue

            min_moisture = plant["ideal_soil_moisture_percent"]["min"]
            deficit = min_moisture - moisture  # positive means the cell is below its minimum

            if deficit > greatest_deficit:
                greatest_deficit = deficit
                most_deficient_cell = int(cell_id)

        self.current_cell = most_deficient_cell

        if most_deficient_cell is not None:
            self.publish_water_cell(most_deficient_cell)
            self.info(f"Sending water_cell for cell {most_deficient_cell} (deficit: {greatest_deficit:.1f}%)")
        else:
            self.info("All cells have sufficient moisture, nothing to water")

    def cell_still_needs_water(self, cell_id, state, crops):
        """
        Returns True if the given cell is still below its plant's minimum.

        Parameters
        ----------
        cell_id : int
            The cell to check.
        state : dict
            Current moisture per cell (from /get_state).
        crops : dict
            Current crop per cell (from /get_crops).

        Returns
        -------
        bool
            True if the cell exists, has known plant data, and is below its
            minimum moisture. False otherwise.
        """
        key = str(cell_id)
        if key not in state or key not in crops:
            return False

        crop = crops[key]["plant"].lower()
        plant = self.plant_info.get(crop)
        if plant is None:
            return False

        return self.needs_water(plant, state[key]["moisture"])

    def publish_water_cell(self, cell_id):
        """
        Publishes a /water_cell command for the given cell.

        Parameters
        ----------
        cell_id : int
            The cell the robot should water next.
        """
        msg = Int32()
        msg.data = cell_id
        self.water_cell_publisher.publish(msg)

    def significant_rain(self):
        """
        Returns True if it's raining enough to skip watering today.

        Returns
        -------
        bool
            True if rainfall exceeds the configured threshold.
            False if the weather node is unreachable (water as normal).
        """
        weather = self.get_weather()

        if weather is None:
            return False

        rain_mm = weather["water_mm_per_day"]
        threshold = self.config["skip_watering_if_rain_mm_above"]
        return rain_mm > threshold

    def needs_water(self, plant, moisture):
        """
        Returns True if a cell's moisture is below the plant's minimum.

        Parameters
        ----------
        plant : dict
            The plant's record from plant_database.json.
        moisture : float
            The cell's current moisture (0-100).

        Returns
        -------
        bool
        """
        return moisture < plant["ideal_soil_moisture_percent"]["min"]


# We use a manual loop instead of rclpy.spin() because decide() calls ROS
# services and waits for their replies. Calling a service from inside a spin
# callback causes a deadlock — the callback waits for the reply, but the reply
# can't arrive because spin is blocked waiting for the callback to finish.
# Running decide() here, outside any callback, avoids this problem.
def main(args=None):
    node = DecisionMaker()

    try:
        while node.ok():
            node.process_once()  # receive incoming messages (may set should_decide)
            node.decide()        # runs only if should_decide is True
    except KeyboardInterrupt:
        pass
    node.destroy()
