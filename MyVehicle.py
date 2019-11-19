from dronekit import * #connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command, Vehicle, wait_for
from pymavlink import mavutil # Needed for command message definitions
from shapely.geometry.point import Point
import time
import math

from ConeTypeSensor import ConeTypeSensor
from GeoFenceSensor import GeoFenceSensor
from PhysicalObject import PhysicalObject
from CameraSensor import CameraSensor
from TargetGenerator import TargetGenerator
from geometric_helper_functions import *

class MyVehicle(Vehicle):
    def __init__(self, *args):
        super(MyVehicle, self).__init__(*args)

        # save the initial heading, geofence and targets are defined relative to the initial frame (FLU)
        self.initial_heading = self.heading

        # Setup sensors
        print('Setting up sensors...')
        self.geo_sensor = GeoFenceSensor(self, range=20)
        self.target_sensor = ConeTypeSensor(self, range=100)
        self.cameraSensor = CameraSensor(self, range=5)
        self.target_seen = False

            # self.local_NED_coordinates = Point(self.location.local_frame.east,  self.location.local_frame.north) # makes testing without SITL easier
        # self.geo_sensor.update_readings()
        # self.get_home_loc()
        

        self.seen_targets = []

        print('MyVehicle Init finished')

        # @self.on_attribute('home_location')
        # def decorated_mode_callback(self, attr_name, value):

    def set_attribute_listeners(self):
        @self.on_attribute('location')   
        def decorated_mode_callback(self, attr_name, value):
            # update relative gps-coordinates
            # TODO is needed?
            if self.location.local_frame.east is not None:
                self.local_NED_coordinates = Point(self.location.local_frame.east,  self.location.local_frame.north) # makes testing without SITL easier
            
            if self.geo_sensor.points:
                self.geo_sensor.update_readings()
            # check if any targets are seen and generate new ones if they are
            s, uns = self.cameraSensor.update_readings(self.environment.targets)
            if len(s)>0:
                # keep track of the targets this vehicle has seen
                self.seen_targets.append(s)
                # update environment with remaining targets
                self.environment.set_targets(uns)
                # use this as a flag to determine in which timestamp a target is seen
                self.target_seen = True

    def get_home_loc(self):
        # Get Vehicle Home location - will be `None` until first set by autopilot
        print(" Waiting for home location ...")
        cmds = self.commands
        cmds.download()
        cmds.wait_ready()
        if not self.home_location:
            print(" Waiting for home location ...")

        # Get the home location
        print('Home location set at: {}'.format(self.home_location))
        return self.home_location

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        if self.location.global_relative_frame.alt > 10:
            print('Plane already in the air')
            pass
        cmds = self.commands

        print(" Clear any existing commands")
        cmds.clear() 

        # create command using mavlink mission command message 
        # http://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-takeoff
        # param1=pitch angle at takeoff, param7=desired altitude
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        self.home_location.lat, self.home_location.lon, aTargetAltitude)) # first command in the mission is ignored for some reason, see https://github.com/dronekit/dronekit-python/blob/source-system-filtering/examples/avoidance/avoidance-manytests-plane.py#L376
        
        cmds.add(Command( 0, 0, 0, 
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 0, aTargetAltitude))

        print(" Upload new commands to vehicle")
        cmds.upload()

        print ("Param: %s" % self.parameters['ARMING_CHECK'])
        # Disable all arming checks since we are flying in sitl
        # self.parameters['ARMING_CHECK']=1
        

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        # Plane can only take of in auto-mode and with a mission loaded
        while not self.mode == VehicleMode('AUTO'):
            # Set the vehicle into auto mode
            self.mode = VehicleMode("AUTO")

        while not self.armed:
            print(" Waiting for arming...")
            self.armed = True
            time.sleep(1)
        print('Motor armed')

        print('System status: %s' % (self.system_status))
        
        print("Taking off!")
        # self.groundspeed=200.0
        while True:
            print (" Altitude: ", self.location.global_relative_frame.alt)
            print('System status: %s' % (self.system_status))
            #Break and return from function just below target altitude.
            if self.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                            yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                            thrust = 0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.send_mavlink(msg)

    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                    yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                    thrust = 0.5, duration = 0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                            yaw_angle, yaw_rate, False,
                            thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                yaw_angle, yaw_rate, False,
                                thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                            0, 0, True,
                            thrust)

    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def send_position_target(self, location, altitude):
        # # MAV_CMD_NAV_WAYPOINT /// arduplane is not recognising the command
        # msg = self.message_factory.command_long_encode(
        #     0, 0, # target system, target component,
        #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        #     2, # confirmation 
        #     0, # hold time, not supported by ardupilot
        #     0.1, # acceptance radius, set very low (in meters)
        #     0, # not supported by ardupilot
        #     0, # not supported by ardupilot
        #     location.lat, 
        #     location.lon, 
        #     altitude 
        # )
        # self.send_mavlink(msg)
        cmds = self.commands

        cmds.clear() 

        # create command using mavlink mission command message 
        # first command might be ignored??? not sure if only for takeoff
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        self.home_location.lat, self.home_location.lon, altitude)) # first command in the mission is ignored for some reason, see https://github.com/dronekit/dronekit-python/blob/source-system-filtering/examples/avoidance/avoidance-manytests-plane.py#L376
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 1, 0, 0,
        location.lat, location.lon, altitude))
        cmds.upload()
        print('sent waypoint location:{}'.format(location))

    # WORKS
    def set_position(self, location, altitude, duration):
        self.mode = VehicleMode("AUTO")
        self.send_position_target(location, altitude)
        start = time.time()
        while time.time() - start < duration:
            self.send_position_target(location, altitude)
            time.sleep(0.1)
        # # Reset attitude, or it will persist for 1s more due to the timeout
        # self.send_position_target(0)

    def set_FLU_position(self, FLU_position, altitude, duration):
        wp_coord = FLU_to_latlon(FLU_position, self.heading, self.location.global_relative_frame)
        print(self.heading)
        print('current location:{}'.format(self.location.global_relative_frame))
        print('waypoint location:{}'.format(wp_coord))
        self.set_position(wp_coord, altitude, duration)

    def control_plane(self, thrust, angle, duration):
        # this shit doesnt work
        # coord = on_half_circle(angle, angle_limit=45, r=300)
        # print('FLU control coord:{}'.format(coord))
        # self.set_FLU_position(coord, 100, 1)
        # scale to 1000-2000 range
        pwm = (angle * 500) + 1500 
        
        self.mode = VehicleMode("CRUISE")
        start = time.time()
        while time.time() - start < duration:
            self.channels.overrides['1'] = int(pwm)
            time.sleep(0.01)


    def set_environment(self, env):
        self.environment = env

    def get_sensor_readings(self):
        return self.geo_sensor.readings + self.target_sensor.readings

    def reset(self):
        self.seen_targets = []
        self.initial_heading = self.heading
        self.home_location = LocationGlobal(self.location.global_frame.lat,
                                            self.location.global_frame.lon,
                                            584.21)
        self.geo_sensor.reset_FLU_geo_fence()  