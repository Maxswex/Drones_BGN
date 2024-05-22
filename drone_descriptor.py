import json

class DroneDescriptor:

    def __init__ (self, ID, purpose):
        self.ID = ID
        self.purpose = purpose


    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)