# based on https://github.com/dronekit/dronekit-python/blob/master/examples/set_attitude_target/set_attitude_target.py

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
from shapely.geometry.point import Point
import time
import math

from ConeTypeSensor import ConeTypeSensor
from GeoFenceSensor import GeoFenceSensor
from PhysicalObject import PhysicalObject

# home coordinates
home_lat = -35.359272
home_lon = 149.163757

geoFenceWaypoints = [
    (-35.360489,	149.162277),
    (-35.360031,	149.167496),
    (-35.365093,	149.168137),
    (-35.365356,	149.163361)
    ]

# Had trouble running SITL with dronekit-sitl python api for a local built binary, running sitl seperately now with sim_vehicle1.py
# print "Start simulator (SITL)"
# import dronekit_sitl
# from dronekit_sitl import SITL
# sitl_bin_path = '/home/robbert/projects/ardupilot/build/sitl/bin/arduplane'
# sitl_args = ['--model', 'plane', ]
# sitl = SITL(path=sitl_bin_path) # load a binary path (optional)

# sitl.launch(sitl_args, verbose=True, await_ready=True, restart=True)

connection_string = '127.0.0.1:14550'

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    cmds = vehicle1.commands

    print(" Clear any existing commands")
    cmds.clear() 

    # create command using mavlink mission command message 
    # http://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-takeoff
    # param1=pitch angle at takeoff, param7=desired altitude
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 0, 0, 0, 0, 0,
    home_lat, home_lon, aTargetAltitude)) # first command in the mission is ignored for some reason, see https://github.com/dronekit/dronekit-python/blob/source-system-filtering/examples/avoidance/avoidance-manytests-plane.py#L376
    
    cmds.add(Command( 0, 0, 0, 
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
     0, 0, 0, 0, 0, 0, 0, 0, aTargetAltitude))

    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 0, 0, 0, 0, 0,
    home_lat, home_lon, aTargetAltitude))

    print(" Upload new commands to vehicle")
    cmds.upload()

    print ("Param: %s" % vehicle1.parameters['ARMING_CHECK'])
    # Disable all arming checks since we are flying in sitl
    # vehicle1.parameters['ARMING_CHECK']=1
    

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle1.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Plane can only take of in auto-mode and with a mission loaded
    # while not vehicle1.mode == VehicleMode('AUTO'):
    # Set the vehicle into auto mode
    vehicle1.mode = VehicleMode("AUTO")

    while not vehicle1.armed:
        print(" Waiting for arming...")
        vehicle1.armed = True
        time.sleep(1)
    print('Motor armed')

    print('System status: %s' % (vehicle1.system_status))
    
    print("Taking off!")
    # vehicle1.groundspeed=200.0
    while True:
        print (" Altitude: ", vehicle1.location.global_relative_frame.alt)
        print('System status: %s' % (vehicle1.system_status))
        #Break and return from function just below target altitude.
        if vehicle1.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
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
        yaw_angle = vehicle1.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle1.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle1.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
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



# Connect to the vehicle1.
print("Connecting to vehicle1 on: %s" % (connection_string,))
vehicle1 = connect(connection_string, wait_ready=True)

# Setup sensors
sensor = ConeTypeSensor(vehicle1)
geoSensor = GeoFenceSensor(vehicle1, geoFenceWaypoints)
# sensor.updateReadings(objects)
vehicle1.gps_coordinates = Point(vehicle1.location.global_relative_frame.lat,  vehicle1.location.global_relative_frame.lon) # makes testing without SITL easier
geoSensor.update_readings()

@vehicle1.on_attribute('location')   
def decorated_mode_callback(self, attr_name, value):
    # sensor.updateReadings(objects)
    vehicle1.gps_coordinates = Point(vehicle1.location.global_relative_frame.lat,  vehicle1.location.global_relative_frame.lon) # makes testing without SITL easier
    print(vehicle1.gps_coordinates)
    geoSensor.update_readings()
    
# Take off, locks untill altitude is reached 
arm_and_takeoff(10.0)

print('Switching to GUIDED mode')
vehicle1.mode = VehicleMode("GUIDED")
# Hold the position for 3 seconds.
# print("Hold position for 3 seconds")
# set_attitude(duration = 3)

# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
set_attitude(yaw_rate = -30, thrust = 0.5, duration = 3)

# Move the drone forward and backward.
# Note that it will be in front of original position due to inertia.
# print("Move forward")
# set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)

# print("Move backward")
# set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)

print('Setting mode: RTL')
vehicle1.mode = VehicleMode("RTL")

# print("Setting LAND mode...")
# vehicle1.mode = VehicleMode("LAND")
# time.sleep(100)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle1.close()

# Shut down simulator if it was started.
# if sitl is not None:
#     sitl.stop()

print("Completed")
