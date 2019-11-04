from shapely.geometry.point import Point
from dronekit import Vehicle

class VehicleCustom(Vehicle):
# not being used right now
    def __init__(self, *args, **kargs):
        if ( 'dummy_heading' and 'dummy' in kargs)
        super(VehicleCustom, self).__init__(*args)