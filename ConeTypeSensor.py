import cmath
import math
from dronekit import LocationGlobal, LocationGlobalRelative
from shapely.affinity import affine_transform

class ConeTypeSensor:
    """cone type sensor
        supplies readings for pysical objects for the 4 directions relative to the plane (1=forward, 2=right, 3=back, 4=left)
    
    """

    def __init__(self, owner, range=1.0):
        self.readings = [range]*4   # readings for the 4 directions relative to the plane (1=forward, 2=right, 3=back, 4=left)
        self.range = range
        self.objects = []
        self.owner  = owner


    def update_readings(self, objects):
        self.objects = objects
        for obj in self.objects: (obj.distance, obj.angle) = self.calc_distance_and_angle(obj.gps_coordinates)

        temp = []
        temp.append(list(filter(lambda x: x.angle <= 135 and x.angle >= 45, self.objects ))) # forward
        temp.append(list(filter(lambda x: x.angle <= 45. or x.angle >= 315., self.objects ))) # right
        temp.append(list(filter(lambda x: x.angle <= 315 and x.angle >= 225, self.objects ))) # back
        temp.append(list(filter(lambda x: x.angle <= 225 and x.angle >= 135, self.objects ))) # left

        for i,t in enumerate(temp):
            if not t: # empty list, default to max range
                self.readings[i] = self.range
            else: # find closest physical object in list
                self.readings[i] = min(t, key= lambda x: x.distance).distance

        print(self.readings)  

    def calc_distance_and_angle(self, p):
        # transform to local coordinates
        p_local = affine_transform(p, [1,0,0,1, -self.owner.gps_coordinates.x, -self.owner.gps_coordinates.y])
        # convert to polar coordinates
        (distance, angle) = cmath.polar(complex(p_local.x, p_local.y)) # angle is in radians
        
        # convert to degrees
        angle = math.degrees(angle)

        # take into account the relative angle of the plane
        angle = angle + self.owner.heading #
        
        # only positive angles
        if angle < 0:
            angle+=360

        # only angles under 360
        if angle > 360:
            angle-=360
        
        return (distance, angle)

    def get_normalized_readings(self):
        return [r/self.range for r in self.readings]


            

    
