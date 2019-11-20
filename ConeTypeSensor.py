import cmath
import math
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from shapely.affinity import affine_transform
from geometric_helper_functions import *

class ConeTypeSensor:
    """cone type sensor
        supplies readings for pysical objects for the 4 directions relative to the plane (1=forward, 2=right, 3=back, 4=left)
    
    """

    def __init__(self, owner, range=1.0):
        self.readings = [0]*4   # readings for the 4 directions relative to the plane (1=forward, 2=right, 3=back, 4=left)
        self.range = range
        self.objects = []
        self.owner  = owner


    def update_readings(self, objects, inversed=True):
        self.objects = objects
        d_and_a = [calc_distance_and_angle(obj, self.owner.location.local_frame, rel_angle=self.owner.heading) for obj in self.objects]

        temp = []
        temp.append(list(filter(lambda x: x[1] <= 45 or x[1] >= 315, d_and_a ))) # forward
        temp.append(list(filter(lambda x: x[1] <= 135 and x[1] >= 45, d_and_a ))) # right
        temp.append(list(filter(lambda x: x[1] <= 225 and x[1] >= 135, d_and_a ))) # back
        temp.append(list(filter(lambda x: x[1] <= 315 and x[1] >= 225, d_and_a ))) # left

        for i,t in enumerate(temp):
            if not t: # empty list, default to max range
                r = self.range
            else: # find closest physical object in list
                closest = min(t, key= lambda x: x[0])[0]
                r = closest if closest<self.range else self.range
            self.readings[i] = r/self.range

        if inversed:
            self.readings = [(r-1) * -1 for r in self.readings]

        return self.readings

    # def calc_distance_and_angle(self, p):
    #     # transform to local coordinates
    #     p_local = affine_transform(p, [1,0,0,1, -1*self.owner.location.local_frame.east, -1*self.owner.location.local_frame.north])
    #     # convert to polar coordinates
    #     (distance, angle) = cmath.polar(complex(p_local.x, p_local.y)) # angle is in radians
        
    #     # convert to degrees
    #     angle = math.degrees(angle)

    #     # take into account the relative angle of the plane
    #     angle = angle + self.owner.heading #
        
    #     # only positive angles
    #     if angle < 0:
    #         angle+=360

    #     # only angles under 360
    #     if angle > 360:
    #         angle-=360
        
    #     return (distance, angle)



            

    
