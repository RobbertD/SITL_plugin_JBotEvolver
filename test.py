from geometric_helper_functions import *
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from LocationLocalFLU import LocationLocalFLU
import cmath
import math
from shapely.affinity import affine_transform, rotate
from shapely.geometry.point import Point
from typing import Union
from DummyVehicle import DummyVehicle
from TargetGenerator import TargetGenerator

# print(latlon_to_xy((11,1), (10,0)))
# print(xy_to_latlon((100,100), (-35.360489,	149.162277)))
# print(FLU_to_latlon((1, 1), 180, (0,0)))

# print(FLU_to_NED(LocationLocalFLU(1,-1,0), 0, LocationLocal(0,0,0))) # LocationLocal:north=1.0,east=1.0,down=0
# to_latlon = FLU_to_latlon(LocationLocalFLU(100,0,0), 30, LocationGlobal(0, 0, 0))
# to_ned = FLU_to_NED(LocationLocalFLU(100,0,0), 30, LocationLocal(0, 0, 0))
# back_to_ned = latlon_to_NED(to_latlon, LocationGlobal(0, 0, 0))
# print(to_latlon) #0.0007818768, 0.00045143838
# print(to_ned)

# print(back_to_ned)
# print(on_half_circle(0, 45, 100))

# print(rotate(Point(1,1), 45, origin=(0,0)))
rel_geofence_waypoints = [
    (500, 500),
    (500, -500),
    (-500, -500),
    (-500, 500),
]
dv = DummyVehicle(LocationLocal(400, 400, 0), heading=0)

# # test FLU_to_NED
# NED_geofence = [FLU_to_NED(LocationLocalFLU(p[1], p[0], 0), 0, LocationLocal(400, 400, 0)) for p in rel_geofence_waypoints]
# latlon_geofence = [NED_to_latlon(g, LocationGlobal(-35, 145, 0)) for g in NED_geofence]
# print('Geofence set with the following coordinates: \n{}\n{}\n{}\n{} '.format(latlon_geofence[0], latlon_geofence[1], latlon_geofence[2], latlon_geofence[3]))
# print('Geofence set with the following coordinates: \n{}\n{}\n{}\n{} '.format(NED_geofence[0], NED_geofence[1], NED_geofence[2], NED_geofence[3]))
# NED_geofence = [(p.east, p.north) for p in NED_geofence]

# # test NED_to_latlon
# latlon = NED_to_latlon(LocationLocal(400,400,0), LocationGlobal(-35, 145, 0)) 
# print(latlon)

# test set flu fence
NED_geofence = [FLU_to_NED(LocationLocalFLU(p[1], p[0], 0), 8, LocationLocal(north=1503.35546875,east=-204.7209930419922, down=0) ) for p in rel_geofence_waypoints]
print('Geofence set with the following coordinates: \n{}\n{}\n{}\n{} '.format(NED_geofence[0], NED_geofence[1], NED_geofence[2], NED_geofence[3]))

# # test coneType Sensor
# target_gen = TargetGenerator(rel_geofence_waypoints)
# targets = target_gen.generate_targets(1, dv)
# print(dv.cone_sensor.update_readings([LocationLocal(300,500,0)]))

# # test geofence
# print(dv.location.local_frame)

# dv.geo_sensor.set_geo_fence(rel_geofence_waypoints)

# print(dv.geo_sensor.update_readings())





