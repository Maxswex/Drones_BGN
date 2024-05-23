import json
import random
import time
from drones_geo_location import GeoLocation

class DroneTelemetryData:

    def __init__(self):
        self.batteryLevel = 100.0
        self.geoLocation = GeoLocation(0.0, 0.0, 0.0)
        #self.timestamp = int(time.time())

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    def update_measurement(self):
        random_latitude = 10.0 + random.uniform(0.0, 10.0)
        random_longitude = 40.0 + random.uniform(0.0, 10.0)
        random_altitude = 20 + random.uniform(0.0, 20.0)
        self.geoLocation = GeoLocation(random_latitude, random_longitude, random_altitude)
        self.batteryLevel = self.batteryLevel - random.uniform(0.0, 5.0)
        #self.timestamp = int(time.time())
