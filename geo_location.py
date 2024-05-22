import json

class GeoLocation:

    def __init__(self, latitude, longitude, altitude ):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)