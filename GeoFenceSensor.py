import cmath
import math
from shapely.geometry.polygon import LinearRing, LineString, Polygon
from shapely.geometry.point import Point
from shapely.ops import nearest_points
from shapely.affinity import affine_transform
from ConeTypeSensor import ConeTypeSensor
from PhysicalObject import PhysicalObject
from pprint import pprint


class GeoFenceSensor(ConeTypeSensor):
    """GeoFence sensor
    
    """
    def __init__(self, owner, waypoints, range=1.0):
        self.set_geo_fence(waypoints)
        super(GeoFenceSensor, self).__init__(owner, range=range)

    def set_geo_fence(self, waypoints):
         # if the last and first waypoint are the same, delete the last one
        if waypoints[0] == waypoints[-1]:
            del waypoints[-1]
            
        # convert to Points
        self.points = [Point(p[0], p[1]) for p in waypoints]

        # create polygon to check if owner is inside fence
        self.fencePolygon = Polygon(waypoints)

        # convert list of points to list of linestrings
        self.to_LineString(self.points)

    def convert_to_PhysObj(self):
        # finds the coordinates of the closest point in every line between waypoints
        # and stores them as PysObj to work with the ConeTypeSensor parent class

        # clear the objects list
        self.objects = []


        # Sensor has 4 slices (front, left, ...), it is possible that the closest point in a slice is at the intersection
        # of a geoFenceline and the edge of a slice. 
        sliceAngles = [45,135,225,315]
        sliceAngles = [x+self.owner.heading for x in sliceAngles]
        sliceLineStrings = []

        for a in sliceAngles:
            p1_complex = cmath.rect(self.range, math.radians(a))

            p1_local = Point(p1_complex.real, p1_complex.imag)

            # go from local to global
            p1_global = affine_transform(p1_local, [1,0,0,1,self.owner.gps_coordinates.x, self.owner.gps_coordinates.y])

            sliceLineStrings.append(LineString([p1_global, self.owner.gps_coordinates]))

        for line in self.lines:
            # add the closest point in every line to the object list
            p1, p2 = nearest_points(line, self.owner.gps_coordinates)
            self.objects.append(PhysicalObject('FencePoint', p1.x, p1.y))

            # check if any of the sliceLineStrings intersect with the line
            for s in sliceLineStrings:
                intersect = s.intersection(line)
                
                if intersect:
                    # only 1 intersect possible between 2 lines
                    self.objects.append(PhysicalObject('FencePoint', intersect.x, intersect.y))

        
    def update_readings(self):
        if self.check_inside_fence():
            self.convert_to_PhysObj()
            return super(GeoFenceSensor, self).update_readings(self.objects)
        else:
            # if owner is outside the geofence set sensorvalues to 0 except the direction to the centroid of the geofence polygon
            distance, angle = super(GeoFenceSensor,self).calcDistanceAndAngle(self.fencePolygon.centroid)
            if angle <= 135 and angle >= 45: self.readings = [self.range,0,0,0]         # front
            elif angle <= 45 and angle >= 315: self.readings = [0,self.range,0,0]    # right
            elif angle <= 315 and angle >= 225: self.readings = [0,0,self.range,0]   # back
            elif angle <= 225 and angle >= 135: self.readings = [0,0,0,self.range]   # left
            print(self.readings)


    def to_LineString(self, points):
        # connect all the waypoints into individual LineStrings
        self.lines = []

        prevPoint = points[-1]
        for p in points:
            self.lines.append(LineString([prevPoint, p]))
            prevPoint = p

    def check_inside_fence(self):
        return self.fencePolygon.contains(self.owner.gps_coordinates)