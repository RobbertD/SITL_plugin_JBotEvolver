from ConeTypeSensor import ConeTypeSensor
from GeoFenceSensor import GeoFenceSensor
from PhysicalObject import PhysicalObject

plane = PhysicalObject('plane', 0.9, 0)
objects = []
objects.append(PhysicalObject('bot', 1, 0))
objects.append(PhysicalObject('bot', -1, 0))
objects.append(PhysicalObject('bot', 0, -1))

waypoints = [(1,0), (-1,0), (0,-1)]
waypoints2 = [(1,1), (-1,1), (-1,-1), (1,-1)]

# for obj in objects:
#     print(obj.gps_coordinates)

# sensor = ConeTypeSensor(plane)
geoSensor = GeoFenceSensor(plane, waypoints2, range=3.0)
# sensor.updateReadings(objects)
# geoSensor.setGeoFence()
geoSensor.update_readings()


