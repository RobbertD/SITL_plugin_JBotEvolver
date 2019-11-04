from shapely.geometry.point import Point


class PhysicalObject:
    """cone type sensor"""
    # i = 12345

    def __init__(self, objType, lat, lon):
        self.gps_coordinates = Point(lat, lon)
        self.objType = objType
        self.heading = 0
        self.angle = None
        self.distance = None


        