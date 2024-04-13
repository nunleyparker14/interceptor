import numpy as np
import launch_files.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV
import launch_files.chap05.model_coef as MC


gravity = MAV.gravity  # gravity constant
Va0 = TF.Va_trim
rho = MAV.rho
sigma = .01  # low pass filter gain for derivative

def wn(tr, zeta=np.sqrt(2.) / 2.):
    return np.pi / (2. * tr * np.sqrt(1.0 - zeta**2))
def kp(wn, a0=0., b0=1.):
    """(wn**2 - a0) / b0"""
    return (wn**2 - a0) / b0
def kd(wn, zeta, a1=0., b0=1.):
    """(2*z*wn - a1) / b0"""
    return (2. * zeta * wn - a1) / b0

#----------roll loop-------------
# get transfer function data for delta_a to phi
tr_roll = 0.1*2.5
zeta_roll = np.sqrt(2.) / 2.
wn_roll = wn(tr_roll, zeta_roll)
roll_kp = kp(wn_roll, b0=MC.a_phi2)
roll_kd = kd(wn_roll, zeta_roll, a1=MC.a_phi1, b0=MC.a_phi2)

#----------course loop-------------
W_course = 7.0
tr_course = tr_roll * W_course
zeta_course = np.sqrt(2.) / 2.
wn_course = wn(tr_course, zeta_course)
course_kp = kd(wn_course, zeta_course, b0=gravity/Va0)
course_ki = kp(wn_course, b0=gravity/Va0)

#----------yaw damper-------------
yaw_damper_p_wo = .45 # page 117
yaw_damper_kr = 0.196 # page 117

#----------pitch loop-------------
tr_pitch = 0.1*2.5
zeta_pitch = .9 # np.sqrt(2.) / 2.
wn_pitch = wn(tr_pitch, zeta_pitch)
pitch_kp = kp(wn_pitch, a0=MC.a_theta2, b0=MC.a_theta3)
pitch_kd = kd(wn_pitch, zeta_pitch, a1=MC.a_theta1, b0=MC.a_theta3)
K_pitch_DC = (pitch_kp * MC.a_theta3) / wn_pitch**2

#----------altitude loop-------------
W_altitude = 20.0
tr_altitude = tr_pitch * W_altitude
zeta_altitude = np.sqrt(2.) / 2.
wn_altitude = wn(tr_altitude, zeta_altitude)
altitude_ki = kp(wn_altitude, b0=K_pitch_DC*Va0)
altitude_kp = kd(wn_altitude, zeta_altitude, b0=K_pitch_DC*Va0)
altitude_zone = 5.

#---------airspeed hold using throttle---------------
tr_airspeed_throttle = 1.0*1.5
zeta_airspeed_throttle = np.sqrt(2.) / 2.
wn_airspeed_throttle = wn(tr_airspeed_throttle, zeta_airspeed_throttle)
airspeed_throttle_ki = kp(wn_airspeed_throttle, b0=MC.a_V2)
airspeed_throttle_kp = kd(wn_airspeed_throttle, zeta_airspeed_throttle, a1=MC.a_V1, b0=MC.a_V2)


# Print all variables
variables = {
    "gravity": gravity,
    "Va0": Va0,
    "rho": rho,
    "sigma": sigma,
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
    "K_pitch_DC": K_pitch_DC,
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

#     gravity: 9.81
# Va0: 25.0
# rho: 1.2682
# sigma: 0.01
# tr_roll: 0.25
# wn_roll: 8.885765876316732
# roll_kp: 0.6032595997853518
# roll_kd: -0.07688109426173695
# tr_course: 1.75
# wn_course: 1.2693951251881048
# course_kp: 4.574912849264298
# course_ki: 4.1064321708749345
# yaw_damper_p_wo: 0.45
# yaw_damper_kr: 0.196
# tr_pitch: 0.25
# wn_pitch: 14.414615682913361
# pitch_kp: -2.9860588924214606
# pitch_kd: -0.5718693841433384
# K_pitch_DC: 0.5189774227955892
# wn_altitude: 0.4442882938158366
# altitude_kp: 0.04842742694534794
# altitude_ki: 0.015213924872376147
# altitude_zone: 5.0
# tr_airspeed_throttle: 1.5
# wn_airspeed_throttle: 1.4809609793861223
# airspeed_throttle_kp: 0.24690761062476424
# airspeed_throttle_ki: 0.267220731969377