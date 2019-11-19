class LocationLocalFLU():
    """
    A local location object.

    The north, east and down are relative to the EKF origin.  This is most likely the location where the vehicle was turned on.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the local frame.

    :param north: Position north of the EKF origin in meters.
    :param east: Position east of the EKF origin in meters.
    :param down: Position down from the EKF origin in meters. (i.e. negative altitude in meters)
    """

    def __init__(self, front, left, up):
        self.front = front
        self.left = left
        self.up = up

    def __str__(self):
        return "LocationLocalFLU:front=%s,left=%s,up=%s" % (self.front, self.left, self.up)

    # def distance_home(self):
    #     """
    #     Distance away from home, in meters. Returns 3D distance if `down` is known, otherwise 2D distance.
    #     """

    #     if self.north is not None and self.east is not None:
    #         if self.down is not None:
    #             return math.sqrt(self.north**2 + self.east**2 + self.down**2)
    #         else:
    #             return math.sqrt(self.north**2 + self.east**2)