#This class is used to save the coordinates of the elements detected by drones
import json
import random
import time
from drones_geo_location import GeoLocation

class SensibleElementsGeoLocation:

    def __init__(self):
        self.geoLocation = GeoLocation(0.0, 0.0, 0.0)
        self.timestamp = int(time.time())

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    def update_measurement(self):
        random_latitude = 10.0 + random.uniform(0.0, 10.0)
        random_longitude = 40.0 + random.uniform(0.0, 10.0)
        random_altitude = 20 + random.uniform(0.0, 20.0)
        self.geoLocation = GeoLocation(random_latitude, random_longitude, random_altitude)
        self.timestamp = int(time.time())
