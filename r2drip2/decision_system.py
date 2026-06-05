from r2drip2.base import Base
import rclpy
import json
from std_srvs.srv import Trigger
from std_msgs.msg import Int32


#File names (relative to the data/ folder; Base knows where that is)
PLANT_DB_PATH = "plant_database.json"
CONFIG_PATH = "system_config.json"

#Tunable defaults 
DEFAULT_WATER_LITERS = 2.5         # amount to water per irrigation action


class DecisionMaker(Base):
    """
    Rule-based irrigation decision node.

    Every time a /water_change is received it runs one decision cycle:
    check the current farm state and weather, then send a /water_cell
    command for the cell most in need of watering.

    Subscribes
    /water_change (Int32) - triggers a new decision cycle

    Publishes
    /water_cell (Int32) - cell ID that the robot should water next

    Calls (ROS services)
    /get_state   - current moisture of all cells (from farm_manager)
    /get_crops   - current crop of all cells (from farm_manager)
    /get_weather - current / forecast weather
    """

    def __init__(self):
        super().__init__('decision_maker')

        # Load static reference data once into member variables
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

        # Subscription — set flag so the main loop calls decide()
        self.should_decide = True  # run once at startup too
        self.create_subscription(Int32, '/water_change', self._water_change_callback, 10)

        self.info("Decision maker node started!")

    # Subscription callback 

    def _water_change_callback(self, msg):
        """Set flag so the main loop runs a decision cycle."""
        self.should_decide = True

    # Service helpers

    def _call_trigger(self, client):
        """
        Call a Trigger service synchronously.

        Returns the response message decoded as a dict, or None if the
        service is unreachable.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.warning("A required service is not available, skipping")
            return None

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return json.loads(response.message)

    def get_weather(self):
        """Call /get_weather and return the result as a dict, or None."""
        return self._call_trigger(self.weather_client)

    def get_state(self):
        """Call /get_state and return moisture data as a dict, or None."""
        return self._call_trigger(self.state_client)

    def get_crops(self):
        """Call /get_crops and return crop data as a dict, or None."""
        return self._call_trigger(self.crops_client)

    # Decision logic

    def decide(self):
        """
        Run one decision cycle.

        Finds the cell with the largest moisture deficit below its minimum
        requirement, and publishes a /water_cell command for it — unless
        significant rain is expected, in which case watering is skipped.
        """
        if self.significant_rain():
            self.info("Significant rain expected; skipping watering")
            return

        state = self.get_state()
        crops = self.get_crops()

        if state is None or crops is None:
            self.warning("Could not reach farm_manager services; skipping decision")
            return

        # Find the most moisture-needy cell
        best_cell = None
        best_deficit = 0

        for cell_id, cell_state in state["cells"].items():
            moisture = cell_state["moisture"]
            crop = crops["cells"][cell_id]["plant"].lower()
            plant = self.plant_info.get(crop)

            if plant is None:
                self.warning(f"No plant data for '{crop}' (cell {cell_id}), skipping")
                continue

            min_moisture = plant["ideal_soil_moisture_percent"]["min"]
            deficit = min_moisture - moisture  # positive means below minimum

            if deficit > best_deficit:
                best_deficit = deficit
                best_cell = int(cell_id)

        if best_cell is not None:
            msg = Int32()
            msg.data = best_cell
            self.water_cell_publisher.publish(msg)
            self.info(f"Sending water_cell for cell {best_cell} (deficit: {best_deficit:.1f}%)")
        else:
            self.info("All cells have sufficient moisture, nothing to water")

    def significant_rain(self):
        weather = self.get_weather()

        if weather is None:
            return False

        rain_mm = weather["water_mm_per_day"]
        threshold = self.config["skip_watering_if_rain_mm_above"]
        return rain_mm > threshold

    def needs_water(self, plant, moisture):
        return moisture < plant["ideal_soil_moisture_percent"]["min"]


# Drive the loop ourselves (not spin) because _call_trigger makes synchronous
# service calls that would deadlock inside a spin callback.
def main(args=None):
    node = DecisionMaker()

    try:
        while node.ok():
            node.process_once()  # process incoming messages (sets should_decide flag)
            if node.should_decide:
                node.should_decide = False
                node.decide()
    except KeyboardInterrupt:
        pass
    node.destroy()
