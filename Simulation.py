from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
from shapely.geometry.point import Point
import time
import math
import random
import sys
from MyVehicle import MyVehicle
from ConeTypeSensor import ConeTypeSensor
from GeoFenceSensor import GeoFenceSensor
from PhysicalObject import PhysicalObject
from CameraSensor import CameraSensor
from TargetGenerator import TargetGenerator
from geometric_helper_functions import *
# from NEAT_controller import SEEN_TARGET_BONUS
import multiprocess as mp
from Const import Const
import socket

class Simulation():
    

    def __init__(self, SITL_connection_string, port, targets_amount=3, speedup=1, logging=0):
        self.count = 0
        self.speedup = speedup
        # self.rel_geofence_waypoints = [
        #     (500, 500),
        #     (500, -500),
        #     (-500, -500),
        #     (-500, 500),
        # ]
        
        self.SITL_connection_string = SITL_connection_string
        self.port = port
        # Connect to the vehicle1.
        print("Connecting to vehicle1 on: %s" % (SITL_connection_string,))
        try:
            self.vehicle1 = connect(SITL_connection_string, wait_ready=True, vehicle_class=MyVehicle)
            print('Connected to {}'.format(SITL_connection_string))
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise
        self.vehicle1.wait_ready(True, timout=100)
        self.vehicle1.wait_ready('home_location', 'location', timout=100)
        self.vehicle1.set_environment(self)
        self.vehicle1.parameters['SIM_SPEEDUP']=speedup
        self.vehicle1.parameters['LOG_BACKEND_TYPE']=logging
        # Setup terrain following
        self.vehicle1.parameters['TERRAIN_ENABLE']=1
        self.vehicle1.parameters['TERRAIN_FOLLOW']=1
        
        # self.vehicle1.geo_sensor.set_flu_points(self.rel_geofence_waypoints)

        print('home: {}'.format(self.vehicle1.location.global_frame))

        # print('Creating targets ...')
        # self.targets_amount = targets_amount
        # self.target_gen = TargetGenerator(self.rel_geofence_waypoints)
        # self.targets = []

        self.vehicle1.set_attribute_listeners()
        # Finally takeoff
        self.vehicle1.arm_and_takeoff(300)
        self.start_socket(port)

    def start_socket(self, port):
        try:
            print('Opening socket on port: {}'.format(port))
            # Create a socket server to listen for a JBotEvolver task
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # https://stackoverflow.com/questions/23224860/how-to-force-os-to-free-port-with-python
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setblocking(True)
            s.bind(('localhost', port))
            s.listen()

            # Start listening to the port to start simulation runs with JBotEvolver
            self.run_sim(s)
        except Exception as e:
            print('Closing socket after error: {}'.format(s.getsockname()))
            print(e)
        finally:
            print('Closing socket: {}'.format(s.getsockname()))
            s.close()
            self.start_socket(port)


    def run_sim(self, sock):
        while True:
            clientsock, addr = sock.accept()
            print('Connected!')
            while clientsock:
                # while True: # 
                msg = self.wait_for_msg(clientsock)

                if msg == '#####reset':
                    self.reset()
                    time.sleep(5/Const.SPEED_UP) # give the simulation some time to set up

                    while True:
                        self.send_location(clientsock)
                        msg = self.wait_for_msg(clientsock)
                        if msg == '######done':
                            # simulation is done, return to waiting for a reset message 
                            print('received done message')
                            break
                        else:
                            angle = float(msg)
                            # print('final received angle: {}'.format(angle))
                            self.vehicle1.control_plane(thrust=1, angle=-angle, duration=(1/(Const.UPDATE_FREQ*Const.SPEED_UP)))
                # break from clientsock
                # break

    def wait_for_msg(self, clientsock):
        self.count += 1
        # print('message: {}'.format(self.count))
        msg = ''
        clientsock.settimeout(2)
        try:
            msg = clientsock.recv(11).decode("utf-8")
            # ignore testing messages, keep waiting
            if msg == '######test': self.wait_for_msg(clientsock)
        except (TimeoutError):
            print('socket timeout')

        # print('received length: {}'.format(len(msg)))
        # print('received msg: {}'.format(msg))
        msg = msg.replace('\n', '')

        return msg
    
    def send_location(self, clientsock):
        loc = self.vehicle1.get_rel_location()
        # round the readings so message length is constant
        sensor_readings = ["{0:.2f}".format(x) for x in loc]
        clientsock.send(bytes(str(sensor_readings) + '\n', 'utf-8'))

    def reset(self):
        # reset the seed
        # random.seed(a=1337)
        # Fly straight for 5 seconds before starting a clean simulation (no effects from previous simulation)
        self.vehicle1.control_plane(thrust=1, angle=0, duration=(5/(Const.SPEED_UP)))
        self.vehicle1.reset()
        # self.set_targets([])


    def set_targets(self, targets):
        self.targets = targets
        self.targets = self.targets + self.target_gen.generate_targets(self.targets_amount - len(self.targets), self.vehicle1)
        self.shortest_start_dist = min([calc_distance_and_angle(t, self.vehicle1.location.local_frame, self.vehicle1.heading)[0] for t in self.targets])
        # print('shortest_start_dist: {}'.format(self.shortest_start_dist))

    def early_stop(self):
        dist,_ = calc_distance_and_angle(self.vehicle1.location.local_frame, self.vehicle1.env_origin)
        return dist > 700

if __name__ == '__main__':
    s = Simulation('127.0.0.1:14550', targets_amount=5)
    # s.vehicle1.arm_and_takeoff_test(50.)
    # s.vehicle1.control_plane(thrust=1, angle=1, duration=10)
    s.vehicle1.control_plane(angle=1, thrust=1, duration=0.5)
    s.vehicle1.control_plane(angle=-1, thrust=1, duration=0.5)
    s.vehicle1.control_plane(angle=1, thrust=1, duration=0.5)
            
    

