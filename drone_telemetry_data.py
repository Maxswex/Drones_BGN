import json
import random
import time
from geo_location import GeoLocation

class DroneTelemetryData:

    def __init__(self):
        self.batteryLevel = 100.0
        self.geoLocation = GeoLocation(0.0, 0.0, 0.0)
        #self.speedKmh = 0.0
        #self.engineTemperature = 0.0
        self.timestamp = int(time.time())

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    def update_measurement(self):
        random_latitude = 10.0 + random.uniform(0.0, 10.0)
        random_longitude = 40.0 + random.uniform(0.0, 10.0)
        self.geoLocation = GeoLocation(random_latitude, random_longitude, 0.0)
        #self.engineTemperature = 80 + random.uniform(0.0, 20.0)
        self.batteryLevel = self.batteryLevel - random.uniform(0.0, 5.0)
        #self.speedKmh = 10 + random.uniform(0.0, 80.0)
        self.timestamp = int(time.time())