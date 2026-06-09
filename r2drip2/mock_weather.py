"""
Generates fake weather data.

Weather values change slightly from the previous tick instead of being
completely random every tick.
"""

from r2drip2.base import Base
from std_srvs.srv import Trigger
import random
import json

REFRESH_SECONDS = 10

class MockWeather(Base):
    """
    ROS node that provides mock weather data.
    
    **Services**
    - /get_weather
    
    Attributes
    ----------
    state : dict
        The current weather state (temperature, humidity, raining, water_mm_per_day)
    srv : service
        The /get_weather service server
    timer : Timer
        Triggers weather updates every REFRESH_SECONDS
    """
    
    def __init__(self):
        super().__init__('weather_service')
        
        self.srv = self.create_service(Trigger, '/get_weather', self.get_weather_callback)

        self.state = {
            "temperature": 20.0,
            "humidity": 50.0,
            "raining": False,
            "water_mm_per_day": 0.0,
        }
        
        self.timer = self.create_timer(
            timer_period_sec=REFRESH_SECONDS,
            callback=self.weather_timer_callback
        )

        self.info("Mock weather node started")
        
    def weather_timer_callback(self):
        """Called every REFRESH_SECONDS to update the weather state."""
        self.generate()
        self.info(f"Weather updated: {self.state}")

    def generate(self):
        """
        Generates the next weather state, slightly changing from the current one.
        Temperature shifts by up to 1 degree per tick.
        Rain tends to stay in its current state rather than changing every tick.
        Humidity is 100 when raining, else fluctuates between 20 and 90.
        Water_mm_per_day is the amount of rainfall, set to 0 when not raining, random between 0.5 and 5.0 when raining.
        """         
        t = self.state["temperature"] + random.uniform(-1.0, 1.0)
        t = max(5.0, min(35.0, t))

        if self.state["raining"]:
            raining = random.random() < 0.85
        else:
            raining = random.random() < 0.05
    
        if raining:
            humidity = 100.0
            water = round(random.uniform(0.5, 5.0), 1)
        else:
            h = self.state["humidity"] + random.uniform(-5.0, 5.0)
            humidity = round(max(20.0, min(90.0, h)), 1)
            water = 0.0
            
        self.state = {
            "temperature": round(t, 1),
            "humidity": humidity,
            "raining": raining,
            "water_mm_per_day": water,
        }
            
    def get_weather_callback(self, request, response):
        """
        Called when a node requests /get_weather.
        Returns the current weather state as a JSON string in response.message.
        """
        response.success = True
        response.message = json.dumps(self.state)
        return response


def main(args=None):
    weatherNode = MockWeather()
    weatherNode.process()
    weatherNode.destroy()


if __name__ == '__main__':
    main()