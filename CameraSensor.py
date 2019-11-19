from geometric_helper_functions import calc_distance_and_angle


class CameraSensor:

    def __init__(self, owner, range):
        self.range = range
        self.owner = owner

    def update_readings(self, objects):
        seen = []
        unseen = []
        for obj in objects:
            (obj.distance, obj.angle) = calc_distance_and_angle(obj, self.owner.location.local_frame, self.owner.heading)
            if obj.distance <= self.range:
                seen.append(obj)
                print('target seen!')
            else:
                unseen.append(obj)
        return seen, unseen