from shapely.geometry.polygon import Polygon
from shapely.geometry.point import Point
from geometric_helper_functions import *
import random

class TargetGenerator:
    def __init__(self, waypoints):
        self.fencePolygon = Polygon(waypoints)
        self.bounds = self.fencePolygon.bounds # returns (minx, miny, maxx, maxy)
        print(self.bounds)
        
    def generate_targets(self, number, vehicle_ref):
        targets = []

        def find_point():
            x = random.uniform(self.bounds[0], self.bounds[2])
            y = random.uniform(self.bounds[1], self.bounds[3])
            return Point(x,y)
        
        while len(targets) < number:
            p = find_point()
            while not self.fencePolygon.contains(p):
                p = find_point()
            targets.append(LocationLocalFLU(front=p.y, left=p.x*-1, up=0))
            

        targets = [FLU_to_NED(t, vehicle_ref.initial_heading, vehicle_ref.location.local_frame) for t in targets]
        [print('Target created at: {}'.format(t)) for t in targets]
        return targets

    def set_fence(self, vehicle_ref):
        self.fencePolygon = Polygon(vehicle_ref.geo_sensor.points)


        


