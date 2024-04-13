"""
final_project
    mavsim_interceptor

author: Parker Nunley

- last update
    04/12/2024 - Parker Nunley


Objective:
t
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
from PyQt5.QtCore import QTimer
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
from models.mav_dynamics_control import MavDynamics
from models.mav_dynamics_control_interceptor import MavDynamicsINT
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
from controllers.autopilot_interceptor import AutopilotINT
from planners.path_follower import PathFollower
from planners.path_follower_interecptor import PathFollowerINT
from parameters.target_parameters import TargParam
from mavsim_python.viewers.INT_and_TARG_viewer import INTAndTARGViewer

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
targ = MavDynamics(SIM.ts_simulation)
interceptor = MavDynamicsINT(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
autopilotINT = AutopilotINT(SIM.ts_simulation)
path_follower = PathFollower()
path_followerINT = PathFollowerINT()
targ_param = TargParam()

# path definition
from message_types.msg_path import MsgPath
from message_types.msg_path_interceptor import MsgPathINT
pathTARG = MsgPath()
pathINT = MsgPathINT()

pathTARG.type = 'line'
pathINT.type = 'line'

hit = False

pathTARG.line_origin = np.array([[np.random.randint(-500, 500),
                                   np.random.randint(-500, 500),
                                     -np.random.randint(800, 1200)]]).T
pathTARG.line_direction = np.array([[np.random.randint(0, 10), np.random.randint(-10, 10), 0.0]]).T
pathTARG.line_direction = pathTARG.line_direction / np.linalg.norm(pathTARG.line_direction)

# Set target to line beginning
targ_param.north0 = pathTARG.line_origin.item(0)
targ_param.east0 = pathTARG.line_origin.item(1)
targ_param.down0 = pathTARG.line_origin.item(2)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 600

# initialize the visualization
app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
int_and_targ_view = INTAndTARGViewer(app=app)  # initialize the mav viewer

prev_distance = 100
prev_chi = 1
timer = 0

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:
    
    if hit is True:
        # -------line follower-------------
        # calculate interceptor line
        # Set the interceptor line origin at its coordinates
        pathINT.line_origin = np.array([interceptor.true_state.north, interceptor.true_state.east, -interceptor.true_state.altitude]).T
        direction_to_target_north = 0 - interceptor.true_state.north
        direction_to_target_east = 0 - interceptor.true_state.east
        direction_to_target_altitude = 500 - interceptor.true_state.altitude
        # Set the interceptor direction line to the direction of the target
        pathINT.line_direction = np.array([direction_to_target_north, direction_to_target_east, direction_to_target_altitude]).T
        pathINT.line_direction = pathINT.line_direction/np.linalg.norm(pathINT.line_direction)
        pathINT.airspeed = 25

        cmd_INT = path_followerINT.update(pathINT, interceptor.true_state,500)  # get command from path follower
 
        # -------autopilot-------------
        delta_INT, commanded_state_INT = autopilotINT.update(cmd_INT, interceptor.true_state)

        # -------physical system-------------
        wind_intensity = 0.2
        current_wind = wind_intensity * wind.update()  # get the new wind vector
        interceptor.update(delta_INT, current_wind)  # propagate the interceptor dynamics

    else:
        # Calculate the distance between the interceptor and the target
        # Calculate the distance between the interceptor and the target using NumPy
        distance = np.linalg.norm(np.array([interceptor.true_state.north, interceptor.true_state.east, -interceptor.true_state.altitude]) -
                            np.array([targ.true_state.north, targ.true_state.east, -targ.true_state.altitude]))


        # Calculate the rate of change of course
        chi = interceptor.true_state.chi
        chi_rate =  np.abs(chi - prev_chi) * 10000  # Calculate rate of change of distance
        print("Rate of change course", chi_rate)

        # Update previous chi for the next iteration
        prev_chi = chi

        # if rate of change of distance is positive, slow down the interceptor, if it is negative, speed up the interceptor
        if chi_rate < 5:
            pathINT.airspeed += 0.2  
            if pathINT.airspeed >= 80:
                pathINT.airspeed = 80
        else:
            pathINT.airspeed -= 0.2
            if pathINT.airspeed < 40:
                pathINT.airspeed = 40

        print("pathINT.airspeed", pathINT.airspeed)

        # -------line follower-------------
        # calculate interceptor line
        # Set the interceptor line origin at its coordinates
        pathINT.line_origin = np.array([interceptor.true_state.north, interceptor.true_state.east, -interceptor.true_state.altitude]).T
        direction_to_target_north = targ.true_state.north - interceptor.true_state.north
        direction_to_target_east = targ.true_state.east - interceptor.true_state.east
        direction_to_target_altitude = targ.true_state.altitude - interceptor.true_state.altitude
        # Set the interceptor direction line to the direction of the target
        pathINT.line_direction = np.array([direction_to_target_north, direction_to_target_east, direction_to_target_altitude]).T
        pathINT.line_direction = pathINT.line_direction/np.linalg.norm(pathINT.line_direction)
        
        cmd_TARG= path_follower.update(pathTARG, targ.true_state)  # get command from path follower
        cmd_INT = path_followerINT.update(pathINT, interceptor.true_state,targ.true_state.altitude)  # get command from path follower
 
        # -------autopilot-------------
        delta, commanded_state = autopilot.update(cmd_TARG, targ.true_state)
        delta_INT, commanded_state_INT = autopilotINT.update(cmd_INT, interceptor.true_state)

        # -------physical system-------------
        wind_intensity = 0.2
        current_wind = wind_intensity * wind.update()  # get the new wind vector
        targ.update(delta,current_wind)  # propagate the target dynamics
        interceptor.update(delta_INT, current_wind)  # propagate the interceptor dynamics

    
    # If the interceptor is within 50 units of the target, make the target lose forces
    if distance < 50.0 or hit is True:
        delta.elevator=0.2
        delta.aileron=-0.2
        delta.rudder=0.2
        delta.throttle=0
        targ.update(delta, current_wind)

        hit = True



        if targ.true_state.altitude <= 5:
            
            targ.true_state.north = np.random.randint(-500, 500)
            targ.true_state.east = np.random.randint(-500, 500)
            targ.true_state.altitude = -np.random.randint(500, 800)
            pathTARG.line_origin = np.array([[targ.true_state.north, targ.true_state.east, targ.true_state.altitude]]).T
            pathTARG.line_direction = np.array([[np.random.randint(-10, 10), np.random.randint(-10, 10), 0.0]]).T
            pathTARG.line_direction = pathTARG.line_direction / np.linalg.norm(pathTARG.line_direction)
            pathTARG.airspeed = 25

            # Set target to line beginning
            targ_param.north0 = targ.true_state.north
            targ_param.east0 = targ.true_state.east
            targ_param.down0 = targ.true_state.altitude

            hit = False
                
    # update the viewer
    int_and_targ_view.update(interceptor.true_state, pathINT, targ.true_state, pathTARG)  # plot planes
    app.processEvents()

    # -------increment time-------------
    sim_time += SIM.ts_simulation


