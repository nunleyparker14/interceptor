"""
msg_path_interceptor
    - messages type for input to path follower

"""
import numpy as np


class MsgPathINT:
    def __init__(self):
        self.type = 'line'
        # desired airspeed along the path
        self.airspeed = 80
        # origin of the straight path line (r)
        self.line_origin = np.array([[0.0, 0.0, 0.0]]).T
        # direction of line -unit vector- (q)
        self.line_direction = np.array([[1.0, 0.0, 0.0]]).T
        # flag that indicates that path has been plotted
        self.plot_updated = False
