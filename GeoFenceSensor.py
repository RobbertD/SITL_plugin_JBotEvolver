import cmath
import math
from shapely.geometry.polygon import LinearRing, LineString, Polygon
from shapely.geometry.point import Point
from shapely.ops import nearest_points
from shapely.affinity import affine_transform
from ConeTypeSensor import ConeTypeSensor
from PhysicalObject import PhysicalObject
from pprint import pprint
from dronekit import LocationLocal
from geometric_helper_functions import *


class GeoFenceSensor(ConeTypeSensor):
    """GeoFence sensor
    
    """
    def __init__(self, owner, range=1.0):
        super(GeoFenceSensor, self).__init__(owner, range=range)
        self.points = []
        self.fencePolygon = None

    def set_geo_fence(self, waypoints):
         # if the last and first waypoint are the same, delete the last one
        if waypoints[0] == waypoints[-1]:
            del waypoints[-1]
            
        # Save as tuples 
        self.points = waypoints

        # create polygon to check if owner is inside fence
        self.fencePolygon = Polygon(waypoints)

        # convert list of points to list of linestrings
        self.to_LineString(self.points)

        # print('Geofence set with the following coordinates: \n{} '.format(waypoints))
        
    def reset_FLU_geo_fence(self):
        # assuming env_origin is reset right before
        NED_geofence = [FLU_to_NED(LocationLocalFLU(p[1], p[0], 0), self.owner.initial_heading, self.owner.env_origin) for p in self.points]
        latlon_geofence = [NED_to_latlon(g, self.owner.location.global_frame) for g in NED_geofence]
        print('Geofence set with the following coordinates: \n{}\n{}\n{}\n{} '.format(latlon_geofence[0], latlon_geofence[1], latlon_geofence[2], latlon_geofence[3]))
        NED_geofence = [(p.east, p.north) for p in NED_geofence]
        self.set_geo_fence(NED_geofence)

    def convert_to_PhysObj(self):
        # finds the coordinates of the closest point in every line between waypoints
        # and stores them as PysObj to work with the ConeTypeSensor parent class

        # clear the objects list
        self.objects = []

        owner_loc = Point(self.owner.location.local_frame.east, self.owner.location.local_frame.north)
        # Sensor has 4 slices (front, left, ...), it is possible that the closest point in a slice is at the intersection
        # of a geoFenceline and the edge of a slice. 
        sliceAngles = [45,135,225,315]
        sliceAngles = [x+self.owner.heading for x in sliceAngles]
        sliceLineStrings = []

        for a in sliceAngles:
            p1_complex = cmath.rect(self.range, math.radians(a))

            p1_local = Point(p1_complex.real, p1_complex.imag)

            # go from local to global
            p1_global = affine_transform(p1_local, [1,0,0,1,owner_loc.x, owner_loc.y])

            sliceLineStrings.append(LineString([p1_global, owner_loc]))

        for line in self.lines:
            # add the closest point in every line to the object list
            p1, p2 = nearest_points(line, owner_loc)
            self.objects.append(LocationLocal(north=p1.y, east=p1.x, down=0))

            # check if any of the sliceLineStrings intersect with the line
            for s in sliceLineStrings:
                intersect = s.intersection(line)
                
                if intersect:
                    # only 1 intersect possible between 2 lines
                    self.objects.append(LocationLocal(north=intersect.y, east=intersect.x, down=0))

        
    def update_readings(self):
        if self.check_inside_fence():
            self.convert_to_PhysObj()
            return super(GeoFenceSensor, self).update_readings(self.objects)
        else:
            # if owner is outside the geofence set sensorvalues to 0 
            # except the direction opposite to the centroid of the geofence polygon
            c = self.fencePolygon.centroid
            distance, angle = calc_distance_and_angle(LocationLocal(north=c.y, east=c.x, down=0), self.owner.location.local_frame)
            if angle <= 135 and angle >= 45: self.readings = [0,0,1,0]         # front
            elif angle <= 45 and angle >= 315: self.readings = [0,0,0,1]    # right
            elif angle <= 315 and angle >= 225: self.readings = [1,0,0,0]   # back
            elif angle <= 225 and angle >= 135: self.readings = [0,1,0,0]   # left
            
            return self.readings


    def to_LineString(self, points):
        # connect all the waypoints into individual LineStrings
        self.lines = []

        prevPoint = points[-1]
        for p in points:
            self.lines.append(LineString([prevPoint, p]))
            prevPoint = p

    def check_inside_fence(self):
        return self.fencePolygon.contains(Point(self.owner.location.local_frame.east, self.owner.location.local_frame.north))