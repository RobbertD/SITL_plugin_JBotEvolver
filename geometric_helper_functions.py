import cmath
import math
from shapely.affinity import affine_transform, rotate
from shapely.geometry.point import Point
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from LocationLocalFLU import LocationLocalFLU
from typing import Union

def calc_distance_and_angle(p: LocationLocal, ref_p: LocationLocal, rel_angle=0):
        # if type(p) is LocationGlobal or LocationGlobalRelative:
        #         p = latlon_to_xy()

        p = Point(p.east, p.north) 
        ref_p = Point(ref_p.east, ref_p.north)
        # transform to local coordinates
        p_local = affine_transform(p, [1,0,0,1, -ref_p.x, -ref_p.y])
        p_local = rotate(p_local, rel_angle) # take into account the relative angle of the plane
        # convert to polar coordinates
        (distance, angle) = cmath.polar(complex(p_local.x, p_local.y)) # angle is in radians
        
        # convert to degrees
        angle = math.degrees(angle)

        return (distance, angle)

def latlon_to_NED(coordinates: Union[LocationGlobal, LocationGlobalRelative], home_coordinates: Union[LocationGlobal, LocationGlobalRelative]):
        r = 6371000 # meters

        def to_global_NED(point, r):
                lam = point.lon
                phi = point.lat
                return (r * math.radians(phi), r * math.radians(lam) * math.cos(math.radians(phi)))

        c_north, c_east = to_global_NED(coordinates, r)
        h_north, h_east = to_global_NED(home_coordinates, r)
        rel_north = c_north-h_north
        rel_east = c_east-h_east

        return LocationLocal(rel_north, rel_east, coordinates.alt)

def NED_to_latlon(coordinates: LocationLocal, home_coordinates: Union[LocationGlobal, LocationGlobalRelative]):
        # latitiude corresponds to North, longitude to East
        r = 6371000 # meters

        d_lon = coordinates.east / (r*math.cos(math.radians(home_coordinates.lat)))
        d_lat = coordinates.north / r

        lat = math.degrees(d_lat) + home_coordinates.lat
        lon = math.degrees(d_lon) + home_coordinates.lon

        return LocationGlobalRelative(lat, lon, alt=coordinates.down*-1)

def FLU_to_NED(p_FLU: LocationLocalFLU, relative_angle, ref_NED_coordinates: LocationLocal):
        p = Point(p_FLU.left*-1, p_FLU.front)
        # print(p)
        # print(relative_angle)
        p_rotated = rotate(p, relative_angle*-1, origin=(0,0))
        # print(p_rotated)
        p_translated = affine_transform(p_rotated, [1,0,0,1, ref_NED_coordinates.east, ref_NED_coordinates.north])

        return LocationLocal(north=p_translated.y, east=p_translated.x, down=p_FLU.up * -1)

def FLU_to_latlon(p: LocationLocalFLU, relative_angle, ref_global_coordinates: Union[LocationGlobal, LocationGlobalRelative]):
        # ref_global_coordinates should be the LocationGlobal of the plane
        p_NED = FLU_to_NED(p, relative_angle, LocationLocal(0,0,0))
        print('ned coordinates: {}'.format(p_NED))
        return NED_to_latlon(p_NED, ref_global_coordinates)

def on_half_circle(angle, angle_limit, r):
        # angle should be from -1 to 1, angle limit in degrees (0-90)
        angle = angle * angle_limit
        complx = cmath.rect(r, math.radians(angle)) # angle is in radians

        return LocationLocalFLU(complx.real, complx.imag*-1, 0)




        



