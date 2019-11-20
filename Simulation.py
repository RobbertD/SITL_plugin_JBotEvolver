from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
from shapely.geometry.point import Point
import time
import math
import random

from MyVehicle import MyVehicle
from ConeTypeSensor import ConeTypeSensor
from GeoFenceSensor import GeoFenceSensor
from PhysicalObject import PhysicalObject
from CameraSensor import CameraSensor
from TargetGenerator import TargetGenerator
from geometric_helper_functions import *
# from NEAT_controller import SEEN_TARGET_BONUS


class Simulation():
    
    def __init__(self, targets_amount=3):
        
        self.rel_geofence_waypoints = [
            (500, 500),
            (500, -500),
            (-500, -500),
            (-500, 500),
        ]

        connection_string = '127.0.0.1:14550'
        # Connect to the vehicle1.
        print("Connecting to vehicle1 on: %s" % (connection_string,))
        self.vehicle1 = connect(connection_string, wait_ready=True, vehicle_class=MyVehicle)
        self.vehicle1.wait_ready(True, timout=100)
        self.vehicle1.wait_ready('location', timout=100)
        self.vehicle1.set_environment(self)
        self.vehicle1.geo_sensor.set_geo_fence(self.rel_geofence_waypoints)
        
        # self.home_alt = None
        self.vehicle1.initial_heading = self.vehicle1.heading
        # if home_alt is None:
        #     home_alt = self.vehicle1.location.global_frame.alt
        print('home: {}'.format(self.vehicle1.location.global_frame))
        # set the home location,
        # is useful when a simulation is already in the air when NEAT_controller.py is run
        self.vehicle1.home_location = LocationGlobal(self.vehicle1.location.global_frame.lat,
                                                     self.vehicle1.location.global_frame.lon,
                                                     584.21)
        # give the simulation some time to send and receive the mavlink message
        time.sleep(1)
        # update the home loc on dronekits side
        self.vehicle1.get_home_loc()

        print('Creating targets ...')
        self.targets_amount = targets_amount
        self.target_gen = TargetGenerator(self.rel_geofence_waypoints)
        self.targets = []

        self.vehicle1.set_attribute_listeners()
        # Finally takeoff
        self.vehicle1.arm_and_takeoff(50)

    def run_sim(self):
        # reset the seed
        random.seed(a=1337)
        self.vehicle1.reset()
        self.set_targets([])


    def set_targets(self, targets):
        self.targets = targets
        self.targets = self.targets + self.target_gen.generate_targets(self.targets_amount - len(self.targets), self.vehicle1)
        self.shortest_start_dist = min([calc_distance_and_angle(t, self.vehicle1.location.local_frame, self.vehicle1.heading)[0] for t in self.targets])
        print('shortest_start_dist: {}'.format(self.shortest_start_dist))

    def get_fitness_data(self):
        # should be called every timestep
        timestep_fitness = 0
        shortest_dist = min([calc_distance_and_angle(t, self.vehicle1.location.local_frame, self.vehicle1.heading)[0] for t in self.targets])

        # Add a bonus if target is seen
        if self.vehicle1.target_seen:
            timestep_fitness += 100 #SEEN_TARGET_BONUS
            self.vehicle1.target_seen = False
        
        # Only add distance fitness is inside geofence
        if self.vehicle1.geo_sensor.check_inside_fence():
            timestep_fitness += (self.shortest_start_dist - shortest_dist) / self.shortest_start_dist
            # print('timestep fitness: {}'.format(timestep_fitness))aq
        else:
             timestep_fitness = -5
        return timestep_fitness

    def early_stop(self):
        dist,_ = calc_distance_and_angle(self.vehicle1.location.local_frame, LocationLocal(0,0,0))
        return dist > 700
            
    

