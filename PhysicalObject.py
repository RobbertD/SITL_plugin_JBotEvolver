from shapely.geometry.point import Point
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal


class PhysicalObject:
    """cone type sensor"""
    # i = 12345

    def __init__(self, objType, lat, lon):
        self.local_NED_coordinates = LocationLocal(lat, lon, 0)
        self.objType = objType
        self.heading = 0
        self.angle = None # angle that the PysicalObject has relative to the plane
        self.distance = None # distance to the plane


        