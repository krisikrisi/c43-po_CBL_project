"""
Rule-based irrigation decision node: waters the cell furthest below its minimum
moisture (sticking with it until satisfied), driven by a timer and /water_change.
"""

from r2drip2.base import Base
from std_srvs.srv import Trigger  # request: empty | response: bool success, string message (JSON)
from std_msgs.msg import Int32, String

import json  # service replies carry their data as a JSON string


# File names (relative to the data/ folder; Base knows where that is)
PLANT_DB_PATH = "plant_database.json"
CONFIG_PATH = "system_config.json"

# Timer interval (seconds) for re-checking the farm; /water_change drives the
# normal loop. Bigger = less idle polling when there's nothing to water.
DECISION_PERIOD_SEC = 5


class DecisionMaker(Base):
    """
    **Subscriptions**
    - /water_change (String) — a cell's moisture changed; triggers a decision

    **Publishers**
    - /water_cell (Int32) — cell the robot should water next

    **Calls (ROS services)**
    - /get_state — current moisture of all cells (from farm_manager)
    - /get_crops — current crop of all cells (from farm_manager)
    - /get_weather — current weather

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
    timer : Timer
        Fires periodically to start a decision cycle.
    busy : bool
        True while a cycle is running or a watering is in progress.
    target_cell : int or None
        The cell currently being watered, kept until it reaches its minimum.
    state : dict or None
        Moisture-per-cell snapshot fetched during the current cycle.
    crops : dict or None
        Crop-per-cell snapshot fetched during the current cycle.
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

        # busy: True from cycle start until /water_change confirms the watering.
        # Blocks overlapping cycles and double-commanding a cell mid-drive.
        self.busy = False
        self.target_cell = None  # cell we're finishing before moving on
        self.state = None        # moisture snapshot for the current cycle
        self.crops = None        # crop snapshot for the current cycle

        # Decide on every /water_change, plus a timer (mainly the first decision).
        self.timer = self.create_timer(DECISION_PERIOD_SEC, self.start_decision)
        self.create_subscription(String, '/water_change', self.water_change_callback, 10)

        self.info("Decision maker node started!")

    def water_change_callback(self, msg):
        """
        Called when the farm manager reports a moisture change (the robot just
        finished a watering), so we can decide again.

        Parameters
        ----------
        msg : String
            JSON with the cell ID and new moisture value.
        """
        self.busy = False
        self.start_decision()

    # --- Decision cycle (asynchronous) ---

    def start_decision(self):
        """
        Begins one decision cycle (weather → state → crops). No-op while busy.
        """
        if self.busy:
            return
        self.busy = True
        self.request_weather()

    def call(self, client, callback):
        """
        Sends an asynchronous Trigger request and routes the reply to a callback.

        Parameters
        ----------
        client : Client
            The service client to call.
        callback : callable
            Run with the completed Future once the reply arrives.
        """
        client.call_async(Trigger.Request()).add_done_callback(callback)

    def request_weather(self):
        """
        Asks /get_weather, or skips straight to state if no weather node is up.
        Weather is optional: if it's down we just water as normal.
        """
        if self.weather_client.service_is_ready():
            self.call(self.weather_client, self.on_weather)
        else:
            self.request_state()

    def on_weather(self, future):
        """
        Handles the /get_weather reply: skip the cycle if significant rain,
        otherwise fetch the farm state.

        Parameters
        ----------
        future : Future
            The completed /get_weather call.
        """
        weather = self.read_response(future)
        if weather is not None and self.significant_rain(weather):
            self.info("Significant rain expected; skipping watering")
            self.busy = False
            return
        self.request_state()

    def request_state(self):
        """Asks /get_state, or aborts the cycle if farm_manager isn't reachable."""
        if not self.state_client.service_is_ready():
            self.warning("farm_manager /get_state not available; skipping decision")
            self.busy = False
            return
        self.call(self.state_client, self.on_state)

    def on_state(self, future):
        """
        Stores the moisture snapshot, then fetches the crops.

        Parameters
        ----------
        future : Future
            The completed /get_state call.
        """
        self.state = self.read_response(future)
        if self.state is None:
            self.busy = False
            return
        self.request_crops()

    def request_crops(self):
        """Asks /get_crops, or aborts the cycle if farm_manager isn't reachable."""
        if not self.crops_client.service_is_ready():
            self.warning("farm_manager /get_crops not available; skipping decision")
            self.busy = False
            return
        self.call(self.crops_client, self.on_crops)

    def on_crops(self, future):
        """
        Stores the crop snapshot and runs the actual watering decision.

        Parameters
        ----------
        future : Future
            The completed /get_crops call.
        """
        self.crops = self.read_response(future)
        if self.crops is None:
            self.busy = False
            return
        self.choose_cell()

    def read_response(self, future):
        """
        Returns the JSON body of a completed Trigger call as a dict, or None.

        Parameters
        ----------
        future : Future
            A completed service call.

        Returns
        -------
        dict or None
            The parsed reply, or None if the call failed.
        """
        try:
            response = future.result()
        except Exception as exc:
            self.warning(f"Service call failed: {exc}")
            return None
        if response is None or not response.success:
            return None
        return json.loads(response.message)

    # --- Watering decision ---

    def choose_cell(self):
        """
        Picks the cell to water and publishes /water_cell: stick with the current
        target until it's satisfied, then move to the cell furthest below minimum.
        """
        # Finish the cell we already started on before moving to another one.
        if self.target_cell is not None and self.does_current_cell_need_water():
            self.publish_water_cell(self.target_cell)
            self.info(f"Continuing to water cell {self.target_cell}")
            return

        # Current cell is satisfied (or we don't have one) — pick the next-worst.
        most_deficient_cell = None
        greatest_deficit = 0

        for cell_id, cell_state in self.state.items():
            moisture = cell_state["moisture"]
            crop = self.crops[cell_id]["plant"].lower()
            plant = self.plant_info.get(crop)

            if plant is None:
                self.warning(f"No plant data for '{crop}' (cell {cell_id}), skipping")
                continue

            min_moisture = plant["ideal_soil_moisture_percent"]["min"]
            deficit = min_moisture - moisture  # positive means the cell is below its minimum

            if deficit > greatest_deficit:
                greatest_deficit = deficit
                most_deficient_cell = int(cell_id)

        had_target = self.target_cell is not None
        self.target_cell = most_deficient_cell

        if most_deficient_cell is not None:
            self.publish_water_cell(most_deficient_cell)
            self.info(f"Sending water_cell for cell {most_deficient_cell} (deficit: {greatest_deficit:.1f}%)")
        else:
            # Nothing to water: clear busy; log only on the transition (no idle spam).
            self.busy = False
            if had_target:
                self.info("All cells have sufficient moisture, nothing to water")

    def does_current_cell_need_water(self):
        """
        Returns True if target_cell is still below its plant's minimum moisture.

        Returns
        -------
        bool
        """
        key = str(self.target_cell)
        if key not in self.state or key not in self.crops:
            return False

        crop = self.crops[key]["plant"].lower()
        plant = self.plant_info.get(crop)
        if plant is None:
            return False

        return self.state[key]["moisture"] < plant["ideal_soil_moisture_percent"]["min"]

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

    def significant_rain(self, weather):
        """
        Returns True if it's raining enough to skip watering today.

        Parameters
        ----------
        weather : dict
            The /get_weather reply.

        Returns
        -------
        bool
            True if rainfall exceeds the configured threshold.
        """
        rain_mm = weather["water_mm_per_day"]
        threshold = self.config["skip_watering_if_rain_mm_above"]
        return rain_mm > threshold


def main(args=None):
    node = DecisionMaker()
    node.process()
    node.destroy()
