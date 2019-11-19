from geometric_helper_functions import *
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from LocationLocalFLU import LocationLocalFLU
import cmath
import math
from shapely.affinity import affine_transform, rotate
from shapely.geometry.point import Point
from typing import Union

# print(latlon_to_xy((11,1), (10,0)))
# print(xy_to_latlon((100,100), (-35.360489,	149.162277)))
# print(FLU_to_latlon((1, 1), 180, (0,0)))

# print(FLU_to_NED(LocationLocalFLU(1,-1,0), 0, LocationLocal(0,0,0))) # LocationLocal:north=1.0,east=1.0,down=0
to_latlon = FLU_to_latlon(LocationLocalFLU(100,0,0), 30, LocationGlobal(0, 0, 0))
to_ned = FLU_to_NED(LocationLocalFLU(100,0,0), 30, LocationLocal(0, 0, 0))
back_to_ned = latlon_to_NED(to_latlon, LocationGlobal(0, 0, 0))
print(to_latlon) #0.0007818768, 0.00045143838
print(to_ned)

print(back_to_ned)
print(on_half_circle(0, 45, 100))

# print(rotate(Point(1,1), 45, origin=(0,0)))