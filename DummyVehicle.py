from GeoFenceSensor import GeoFenceSensor
from ConeTypeSensor import ConeTypeSensor
class DummyVehicle():
    def __init__(self, location, heading, initial_heading=0):
        self.location = Location(location)
        self.heading = heading
        self.initial_heading = initial_heading
        self.geo_sensor = GeoFenceSensor(self, range=1000)
        self.cone_sensor = ConeTypeSensor(self, range=1000)
    
class Location():
    def __init__(self, location):
        self.local_frame = location

