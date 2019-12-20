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
# NED_geofence = [FLU_to_NED(LocationLocalFLU(p[1], p[0], 0), 8, LocationLocal(north=1503.35546875,east=-204.7209930419922, down=0) ) for p in rel_geofence_waypoints]
# print('Geofence set with the following coordinates: \n{}\n{}\n{}\n{} '.format(NED_geofence[0], NED_geofence[1], NED_geofence[2], NED_geofence[3]))

# # test coneType Sensor
# target_gen = TargetGenerator(rel_geofence_waypoints)
# targets = target_gen.generate_targets(1, dv)
# print(dv.cone_sensor.update_readings([LocationLocal(300,500,0)]))

# # test geofence
# print(dv.location.local_frame)

# dv.geo_sensor.set_geo_fence(rel_geofence_waypoints)

# print(dv.geo_sensor.update_readings())

# test NED_TO_FLU
# loc = NED_to_FLU(LocationLocal(-1,-1,0), 180, LocationLocal(0,0,0))
# print(loc)


import matplotlib.pyplot as plt
from shapely.affinity import affine_transform, rotate
from shapely.geometry.point import Point
east = [
    14.99973487854,
12.4080934524536,
12.4080934524536,
9.2569637298584,
-1.37412595748901,
-1.37412595748901,
-15.1694641113281,
-33.4133796691895,
-54.1238098144531,
-76.4237518310547,
-76.4237518310547,
-99.8973083496094,
-123.090690612793,
-123.090690612793,
-145.377532958984,
-165.990173339844,
-184.624603271484,
-200.609588623047,
-200.609588623047,
-212.761047363281,
-220.940536499023,
-225.575500488281,
-226.106521606445,
-222.575271606445,
-222.575271606445,
-214.783050537109,
-203.63996887207,
-189.505386352539

]

north =  [
    379.090393066406,
    355.915954589844,
    355.915954589844,
    332.056121826172,
    310.091400146484,
    310.091400146484,
    289.815948486328,
    273.724395751953,
    261.304016113281,
    252.964111328125,
    252.964111328125,
    250.35856628418,
    251.774307250977,
    251.774307250977,
    257.471862792969,
    267.271514892578,
    280.200103759766,
    296.404602050781,
    296.404602050781,
    315.633270263672,
    336.6064453125,
    358.309539794922,
    380.101745605469,
    401.332672119141,
    401.332672119141,
    421.14111328125,
    439.069763183594,
    454.909118652344

]
nedpoints = zip(east,north)
nedpoints = [Point(i) for i in nedpoints]

# 417.0350646972656,east=15.955523490905762
ref_north= 417.0350646972656
ref_east=15.955523490905762
p_translated = [affine_transform(p, [1,0,0,1, -ref_east, -ref_north]) for p in nedpoints]
p_rotated = [rotate(p, 182, origin=(0,0)) for p in p_translated]

nedpoints= [(p.x,p.y) for p in nedpoints]
p_translated= [(p.x,p.y) for p in p_translated]
p_rotated= [(p.x,p.y) for p in p_rotated]
print(*p_rotated)
plt.scatter(*zip(*nedpoints),label='ned')
plt.scatter(*zip(*p_translated), label='trans')
plt.scatter(*zip(*p_rotated), label='rotated')
plt.legend()
plt.show()