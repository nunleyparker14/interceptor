"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
# import parameters.control_parameters_c as AP
# from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state_interceptor import MsgStateINT
from message_types.msg_delta import MsgDelta
from launch_files.chap05.model_coef import u_trim
# e, a, r, t
u_trim = np.array([[-0.125112, 0.001837, -0.000303, 0.676780]]).T


class AutopilotINT:
    def __init__(self, ts_control):
        # instantiate lateral-directional controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        # self.yaw_damper = TransferFunction(
        #                 num=np.array([[AP.yaw_damper_kr, 0]]),
        #                 den=np.array([[1, AP.yaw_damper_p_wo]]),
        #                 Ts=ts_control)
        self.yaw_damper = TFControl(
                        k=AP.yaw_damper_kr,
                        n0=0.0,
                        n1=1.0,
                        d0=AP.yaw_damper_p_wo,
                        d1=1,
                        Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = PIControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = PIControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = MsgStateINT()

    def update(self, cmd, state):
	
	#### TODO #####
        # lateral autopilot
        phi_c = self.course_from_roll.update(cmd.course_command, state.chi)
        delta_aileron = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_rudder = self.yaw_damper.update(state.r)
        delta_aileron = self.saturate(delta_aileron, -1, 1)
        delta_rudder = self.saturate(delta_rudder, -1, 1)

        # longitudinal autopilot
        theta_c = self.altitude_from_pitch.update(cmd.altitude_command, state.altitude)
        # theta_c = cmd.altitude_command
        delta_elevator = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        delta_throttle = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
        delta_elevator = self.saturate(delta_elevator, -1, 1)
        delta_throttle = self.saturate(delta_throttle, 0, 1)

        chi_c = wrap(cmd.course_command, state.chi)
        
        # Uncomment to use trim values for stable flight 
        # delta_elevator = u_trim.item(0)
        # delta_aileron = u_trim.item(1)
        # delta_rudder = u_trim.item(2)
        # delta_throttle = u_trim.item(3)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_elevator,
                         aileron=delta_aileron,
                         rudder=delta_rudder,
                         throttle=delta_throttle)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = chi_c

        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
    