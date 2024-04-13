import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
import launch_files.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV

#### TODO #####
gravity = MAV.gravity  # gravity constant
Va0 = TF.Va_trim
rho = 1.2682  # density of air
sigma = 0.01  # low pass filter gain for derivative

# repetitive functions

zeta = np.sqrt(2.) / 2.

def wn(tr, zeta=np.sqrt(2.) / 2.):
    return np.pi / (2. * tr * np.sqrt(1.0 - zeta ** 2))

def kp(wn, a0=0., b0=1.):
    return (wn ** 2 - a0) / b0

def kd(wn, zeta=np.sqrt(2.) / 2., a1=0., b0=1.):
    return (2. * zeta * wn - a1) / b0

#----------roll loop-------------
# get transfer function data for delta_a to phi
tr_roll = 0.15
zeta_phi = 0.707
wn_roll = wn(tr_roll)
roll_kp = (wn_roll ** 2) / (TF.a_phi2)
roll_kd = (2 * zeta_phi * wn_roll - TF.a_phi1) / TF.a_phi2

#----------course loop-------------
tr_course = 0.65
course_zeta = 0.7
W_course = 10
wn_course = wn_roll/W_course
course_kp = (2. * course_zeta * wn_course *  Va0) / gravity
course_ki = (wn_course ** 2 * Va0)/gravity

#----------yaw damper-------------
yaw_damper_p_wo = 0.45
yaw_damper_kr = 0.196

#----------pitch loop-------------
tr_pitch =  0.08 # 0.0099 or 0.04
zeta_pitch = 0.9
wn_pitch = wn(tr_pitch)
pitch_kp = (wn_pitch ** 2 - TF.a_theta2) / TF.a_theta3
pitch_kd = (2 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
K_theta_DC = (pitch_kp * TF.a_theta3) / (wn_pitch ** 2)

#----------altitude loop-------------
zeta_altitude = 0.95
W_h = 25.
wn_altitude = wn_pitch / W_h
altitude_kp = (2 * zeta_altitude * wn_altitude) / (K_theta_DC * Va0)
altitude_ki = (wn_altitude ** 2) / (K_theta_DC * Va0)
altitude_zone = 5.

#---------airspeed hold using throttle---------------
tr_airspeed_throttle = 1.8
zeta_airspeed =  0.9
wn_airspeed_throttle = np.pi / (2. * tr_airspeed_throttle \
                                * np.sqrt(np.abs(1.0 - zeta_airspeed ** 2)))
airspeed_throttle_kp = (2 * zeta_airspeed * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
airspeed_throttle_ki = (wn_airspeed_throttle ** 2) / TF.a_V2

# Print all variables
variables = {
    "gravity": gravity,
    "Va0": Va0,
    "rho": rho,
    "sigma": sigma,
    "zeta": zeta,
    "tr_roll": tr_roll,
    "wn_roll": wn_roll,
    "roll_kp": roll_kp,
    "roll_kd": roll_kd,
    "tr_course": tr_course,
    "wn_course": wn_course,
    "course_kp": course_kp,
    "course_ki": course_ki,
    "yaw_damper_p_wo": yaw_damper_p_wo,
    "yaw_damper_kr": yaw_damper_kr,
    "tr_pitch": tr_pitch,
    "wn_pitch": wn_pitch,
    "pitch_kp": pitch_kp,
    "pitch_kd": pitch_kd,
    "K_theta_DC": K_theta_DC,
    #"tr_altitude": tr_altitude,
    "wn_altitude": wn_altitude,
    "altitude_kp": altitude_kp,
    "altitude_ki": altitude_ki,
    "altitude_zone": altitude_zone,
    "tr_airspeed_throttle": tr_airspeed_throttle,
    "wn_airspeed_throttle": wn_airspeed_throttle,
    "airspeed_throttle_kp": airspeed_throttle_kp,
    "airspeed_throttle_ki": airspeed_throttle_ki
}

for var_name, var_value in variables.items():
    print(f"{var_name}: {var_value}")


# wn_roll: 14.809609793861222
# roll_kp: 1.6757211105148668
# roll_kd: -0.012897427527061356
# tr_course: 0.65
# wn_course: 1.4809609793861223
# course_kp: 5.283754768452016
# course_ki: 5.589310454801994
# yaw_damper_p_wo: 0.1
# yaw_damper_kr: 5.0
# tr_pitch: 0.04
# wn_pitch: 55.53603672697958
# pitch_kp: -82.63933661938258
# pitch_kd: -2.7753281292448717
# K_theta_DC: 0.9675942684830712
# wn_altitude: 0.0
# altitude_kp: 0.075
# altitude_ki: 9e-05
# altitude_zone: 0.0
# tr_airspeed_throttle: 1.8
# wn_airspeed_throttle: 2.002029955960189
# airspeed_throttle_kp: 0.430792752723274
# airspeed_throttle_ki: 0.4883419809381891