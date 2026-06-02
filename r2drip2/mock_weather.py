"""
Generates fake weather data.

Weather values change slightly from the previous tick instead of being
completely random every tick.
"""

from r2drip2.base import Base
from std_srvs.srv import Trigger # type of service:   request: empty | response:  bool success, string message (our weather state)

import random
import time
import json # for json.dumps (returning a string cuz its easier than creating a custom service)

class MockWeather(Base):
    
    def __init__(self, refresh_seconds = 90):
        super().__init__('weather_service') #
        self.srv = self.create_service(Trigger, '/get_weather', self.get_weather_callback)

        self.refresh_seconds = refresh_seconds
        self.last_update = time.time()
        self.state = {
            "temperature": 20.0,
            "raining": False,
            "water_mm_per_day": 0.0,
        }

        self.info("Mock weather node started")

    def generate(self):
        t = self.state["temperature"] + random.uniform(-1.0, 1.0)
        t = max(5.0, min(35.0, t))

        if self.state["raining"]:
            raining = random.random() < 0.85
        else:
            raining = random.random() < 0.05
    
        water = round(random.uniform(0.5, 5.0), 1) if raining else 0.0

        self.state = {
            "temperature": round(t, 1),
            "raining": raining,
            "water_mm_per_day": water,
        }

    def update(self):
        if time.time() - self.last_update >= self.refresh_seconds:
            self.generate()
            self.last_update = time.time()
            
    def get_weather_callback(self, request, response):
        self.update()
        response.success = True
        response.message = json.dumps(self.state)

        return response


def main(args=None):
    weatherNode = MockWeather()
    weatherNode.process()
    weatherNode.shutdown()


if __name__ == '__main__':
    main()   