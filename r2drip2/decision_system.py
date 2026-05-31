from r2drip2.base import Base
import rclpy
import json
from datetime import datetime
from std_srvs.srv import Trigger


# --- File names (relative to the data/ folder; Base knows where that is) ---
DIGITAL_FARM_PATH = "digital_farm.json"
PLANT_DB_PATH = "plant_database.json"
CONFIG_PATH = "system_config.json"
SCHEDULE_PATH = "schedule.json"

# --- Tunable defaults ---
DECISION_INTERVAL_SECONDS = 10.0   # how often a decision cycle runs
DEFAULT_WATER_LITERS = 2.5         # amount to schedule for an irrigation action


class DecisionMaker(Base):
    """
    Rule-based irrigation decision node.

    Every `interval` seconds it runs one decision cycle: read the current
    field state, plant requirements and weather, decide which cells need
    watering, and write an irrigation plan to schedule.json.

    Reads (data/)
    -------------
    digital_farm.json   - current moisture (0-100) and crop per cell
    plant_database.json - per-crop moisture requirements
    system_config.json  - thresholds (e.g. rainfall cutoff for skipping)

    Calls (ROS service)
    -------------------
    /get_weather : std_srvs/Trigger
        Returns the current weather as a JSON string in the response message.

    Writes (data/)
    --------------
    schedule.json - the irrigation plan for the robot (format in `decide`)
    """

    #Build the node
    def __init__(self):
        super().__init__('decision_maker')

        self.interval = DECISION_INTERVAL_SECONDS
        self.weather_client = self.create_client(Trigger, '/get_weather')
        self.info("Decision maker node started!")


    #Calls the weather API and returns it as a python dict
    def get_weather(self):
        """
        Call /get_weather and return the weather as a dict, or None if the
        weather node is unreachable.

        Returns
        -------
        dict or None
            Keys: temperature, raining, water_mm_per_day.
        """
        if not self.weather_client.wait_for_service(timeout_sec=2.0):
            self.warning("/get_weather not available")
            return None

        #Send the request and save the placeholder into future
        future = self.weather_client.call_async(Trigger.Request())
        #Waits until the weather node's response arrives into future
        rclpy.spin_until_future_complete(self, future)
        #The actual response
        response = future.result()
        #Returning it back as a python dictionary
        return json.loads(response.message)


    #The decision logic
    def decide(self):
        """
        Run one decision cycle and write the plan to schedule.json.

        schedule.json format
        ---------------------
        {
          "schedule_date": "YYYY-MM-DD",
          "created_at":    "<ISO timestamp>",
          "actions": [
            {
              "cell_id":             "0".."8",
              "plant_name":          str,
              "action_type":         "irrigation" | "skip_irrigation",
              "water_amount_liters": float,
              "reason":              str
            }
          ]
        }
        """
        #Get the info from the json files
        farm = self.read_json(DIGITAL_FARM_PATH)
        plants = self.read_json(PLANT_DB_PATH)
        configuration = self.read_json(CONFIG_PATH)

        #Build a "lookup" table from the plant dictionary (just to search easier)
        plant_info = {
            p["name"].lower(): p for p in plants["plants"]
        }

        #Action queue
        actions = []
        #Checking if it rains for the whole farm
        rains = self.significant_rain(configuration)

        #Looping over the cells
        for cell_id, cell in farm["cells"].items():
            crop = cell["plant"]
            moisture = cell["moisture"]
            plant = plant_info.get(crop.lower())

            #If there is no data for the cell, we just skip it
            if plant is None:
                self.warning(f"No plant data for crop '{crop}' (cell {cell_id})")

            #If it rains we also skip it
            elif rains:
                actions.append({
                    "cell_id": cell_id,
                    "plant_name": crop,
                    "action_type": "skip_irrigation",
                    "water_amount_liters": 0,
                    "reason": "significant amount of rain today"
                })
            #Otherwise we water the plant and update the action list
            elif self.needs_water(plant, moisture):
                actions.append({
                    "cell_id": cell_id,
                    "plant_name": crop,
                    "action_type": "irrigation",
                    "water_amount_liters": DEFAULT_WATER_LITERS,
                    "reason": "Moisture below minimum"
                })

        #This is basically what will eventually get to the robot
        schedule = {
            "schedule_date": datetime.now().date().isoformat(),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "actions": actions,
        }

        self.write_json(SCHEDULE_PATH, schedule)
        self.info(f"schedule at {len(actions)} actions")


    def needs_water(self, plant, moisture):
        """
        Return True if a cell's moisture is below the plant's minimum.

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


    def significant_rain(self, config):
        """
        Return True if enough rain is expected to skip watering.

        Parameters
        ----------
        config : dict
            Contents of system_config.json (for the rainfall threshold).

        Returns
        -------
        bool
            False if the weather node is unreachable (water as normal).
        """
        weather = self.get_weather()

        #If weather node is not reachable we just water as normal
        if weather is None:
            return False

        #If it rains more than the set threshold, return True so we skip watering
        rain_mm = weather["water_mm_per_day"]
        threshold = config["skip_watering_if_rain_mm_above"]
        return rain_mm > threshold


#Loops every `interval` seconds, calls decide(), stops on Ctrl+C.
#We drive the loop ourselves (not spin) because get_weather makes a
#synchronous service call, which would deadlock inside spin.
def main(args=None):
    node = DecisionMaker()

    try:
        while node.ok():
            node.decide()
            node.sleep(node.interval)
    except KeyboardInterrupt:
        pass
    node.destroy()
