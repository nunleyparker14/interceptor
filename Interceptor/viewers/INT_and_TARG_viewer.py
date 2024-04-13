"""
mavsim_python: interceptor and target viewer for final project
    - Parker Nunley, PUP, 2024
    - Update history:
        4/15/2019
"""
import numpy as np
import pyqtgraph.opengl as gl
from viewers.draw_mav import DrawMav
from viewers.draw_interceptor import DrawINT
from viewers.draw_path import DrawPath
from message_types.msg_state import MsgState
from message_types.msg_state_interceptor import MsgStateINT
from message_types.msg_path import MsgPath
from message_types.msg_path_interceptor import MsgPathINT

from time import time

class INTAndTARGViewer:
    def __init__(self, app, ts_refresh=1./24.):
        self.scale = 3000
        self.app = app
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Interceptor and Target Viewer')
        grid = gl.GLGridItem()
        grid.scale(self.scale/20, self.scale/20, self.scale/20)
        self.window.addItem(grid)
        self.window.setBackgroundColor('k')

        self.window.setGeometry(0, 0, 1500, 1000)

        center = self.window.cameraPosition()
        center.setX(0)
        center.setY(0)
        center.setZ(0)

        self.window.setCameraPosition(pos=center, distance=4000, elevation=20, azimuth=-90)
        
        self.window.show()
        self.window.raise_()
        self.plot_initialized = False
        self.interceptor_plot = []
        self.target_plot = []
        self.interceptor_path_plot = []
        self.target_path_plot = []
        self.ts_refresh = ts_refresh
        self.t = time()
        self.t_next = self.t        

    def update(self, 
               interceptor_state: MsgStateINT, 
               pathINT: MsgPathINT,
               target_state: MsgState,
               pathTARG: MsgPath,
               ):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])

        # Extract the position vectors from the states
        interceptor_position = np.array([interceptor_state.north, interceptor_state.east, interceptor_state.altitude])
        target_position = np.array([target_state.north, target_state.east, target_state.altitude])
        # Calculate the vector between the two positions
        position_difference = interceptor_position - target_position
        # Calculate the Euclidean distance (length) between the two points
        distance = np.linalg.norm(position_difference)   
            
            
        if not self.plot_initialized:   
            self.interceptor_plot = DrawINT(interceptor_state, self.window)
            self.target_plot = DrawMav(target_state, self.window)
            self.interceptor_path_plot = DrawPath(pathINT, red, self.window)
            # self.target_path_plot = DrawPath(pathTARG, blue, self.window)
            self.plot_initialized = True
            pathINT.plot_updated = True
            pathTARG.plot_updated = True
        else:
            t = time()
            if t-self.t_next > 0.0:
                self.interceptor_plot.update(interceptor_state)
                self.target_plot.update(target_state)
                self.interceptor_path_plot.update(pathINT,red, distance)
                self.t = t
                self.t_next = t + self.ts_refresh

            if not pathINT.plot_updated:
                # self.interceptor_path_plot.update(pathINT,red, distance)
                pathINT.plot_updated = True

            if not pathTARG.plot_updated:
                # self.target_path_plot.update(pathTARG, red, distance)
                pathTARG.plot_updated = True
        
        self.app.processEvents()
    