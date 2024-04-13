"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import numpy as np
from models.mav_dynamics_interceptor import MavDynamicsINT as MavDynamicsForces
# load message types
from message_types.msg_state_interceptor import MsgStateINT
from message_types.msg_delta import MsgDelta
import parameters.interceptor_parameters as MAV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler


class MavDynamicsINT(MavDynamicsForces):
    def __init__(self, Ts):
        super().__init__(Ts)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''


        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]

        ##### TODO #####
        # convert wind vector from world to body frame (self._wind = ?)
        R_wind_body = quaternion_to_rotation(self._state[6:10]).T
        self._wind = (R_wind_body @ steady_state) + gust

        # velocity vector relative to the airmass ([ur , vr, wr]= ?)
        V_wind_body = self._state[3:6]
        ur, vr, wr = V_wind_body - self._wind
        ur = ur.item(0)
        vr = vr.item(0)
        wr = wr.item(0)

        # compute airspeed (self._Va = ?)
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)

        # compute angle of attack (self._alpha = ?)
        self._alpha = np.arctan2(wr,ur)

        # compute sideslip angle (self._beta = ?)
        tmp = np.sqrt(ur**2 + wr**2)
        if tmp == 0:
            self._beta = np.sign(vr)*np.pi/2
        else:
            self._beta = np.arcsin(np.clip(vr / tmp, -np.pi, np.pi))


    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """

        delta_a = delta.aileron
        delta_e = delta.elevator
        delta_r = delta.rudder
        delta_t = delta.throttle

        ##### TODO ######
        # extract states (phi, theta, psi, p, q, r)
        phi, theta, psi = quaternion_to_euler(self._state[6:10])

        p = self._state[10][0]
        q = self._state[11][0]
        r = self._state[12][0]

        # compute gravitational forces ([fg_x, fg_y, fg_z])

        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)

        stateQuaternion = np.array([[e0,e1,e2,e3]])

        gravitationalForces_vehicleFrame = np.array([[0],
                                                     [0],
                                                     [MAV.M * MAV.gravity]])

        bodyToInertial_rotation = quaternion_to_rotation(stateQuaternion)
        inertialToBody_rotation = np.transpose(bodyToInertial_rotation)

        gravitationalForces_bodyFrame = inertialToBody_rotation @ gravitationalForces_vehicleFrame

        # fx = gravitationalForces_bodyFrame.item(0)
        # fy = gravitationalForces_bodyFrame.item(1)
        # fz = gravitationalForces_bodyFrame.item(2)

        fx = -MAV.mass * MAV.gravity * np.sin(theta)
        fy = MAV.mass * MAV.gravity * np.cos(theta) * np.sin(phi)
        fz = MAV.mass * MAV.gravity * np.cos(theta) * np.cos(phi)

        # compute Lift and Drag coefficients (CL, CD)

        # This is where I can input the blending function comes into play
        M = 20
        self.sig_alpha = (1 + np.exp(-M*(self._alpha - MAV.alpha0)) + np.exp(M*(self._alpha + MAV.alpha0)))\
                     / ((1 + np.exp(-M*(self._alpha - MAV.alpha0))) * (1 + np.exp(M*(self._alpha + MAV.alpha0))))
        
        # these are the linear models...
        CL = MAV.C_L_0 + MAV.C_L_alpha * self._alpha
        CD = MAV.C_D_0 + MAV.C_D_alpha * self._alpha

        # These are the NON-linear models...
        CL_alpha = ((1-self.sig_alpha) * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha))\
                + self.sig_alpha * ((2 * np.sign(self._alpha)) * np.sin(self._alpha)**2 * np.cos(self._alpha))
        CD_alpha  = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * self._alpha)**2)\
                    / (np.pi * MAV.e * MAV.AR)

        # compute Lift and Drag Forces (F_lift, F_drag)
        qbar = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing
        F_lift = qbar * (CL_alpha + MAV.C_L_q * MAV.c * q / (2 * self._Va) + MAV.C_L_delta_e * delta_e)
        F_drag = qbar * (CD_alpha + MAV.C_D_q * MAV.c * q / (2 * self._Va) + MAV.C_D_delta_e * delta_e)

        
        # compute longitudinal forces in body frame (fx, fz)
        # Define the rotation matrix
        rotation_matrix = np.array([[np.cos(self._alpha), -np.sin(self._alpha)],
                                    [np.sin(self._alpha), np.cos(self._alpha)]])

        # Perform the matrix multiplication without flattening
        force_matrix = np.array([-F_drag, -F_lift]).T

        # Perform the matrix multiplication without flattening
        force_matrix_body_frame = rotation_matrix @ force_matrix

        # Extract the components directly
        
        fx += force_matrix_body_frame.item(0)
        fz += force_matrix_body_frame.item(1)

        # compute lateral forces in body frame (fy)
        fy += 0.5 * MAV.rho * self._Va**2 * MAV.S_wing \
        * (MAV.C_Y_0 + MAV.C_Y_beta * self._beta \
        + MAV.C_Y_p * MAV.b * p / (2 * self._Va) \
        + MAV.C_Y_r * MAV.b * r / (2 * self._Va) \
        + MAV.C_Y_delta_a * delta_a \
        + MAV.C_Y_delta_r * delta_r)

        # compute longitudinal torque in body frame (My) pitch
        # m
        My = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.c \
        * (MAV.C_m_0 + MAV.C_m_alpha * self._alpha + MAV.C_m_q * MAV.c * q / (2 * self._Va) \
        + MAV.C_m_delta_e * delta_e)

        # compute lateral torques in body frame (Mx, Mz) roll and yaw
        # l
        Mx = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.b\
        * (MAV.C_ell_0 + MAV.C_ell_beta * self._beta \
        + MAV.C_ell_p * MAV.b * p / (2 * self._Va) \
        + MAV.C_ell_r * MAV.b * r / (2 * self._Va) \
        + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r * delta_r)

        # n
        Mz = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.b\
        * (MAV.C_n_0 + MAV.C_n_beta * self._beta \
        + MAV.C_n_p * MAV.b * p / (2 * self._Va) \
        + MAV.C_n_r * MAV.b * r / (2 * self._Va) \
        + MAV.C_n_delta_a * delta_a + MAV.C_n_delta_r * delta_r)

        # propeller thrust and torque
        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)

        fx += thrust_prop
        Mx -= torque_prop

        fy = fy[0] if isinstance(fy, np.ndarray) and fy.size == 1 else fy
        Mx = Mx[0] if isinstance(Mx, np.ndarray) and Mx.size == 1 else Mx
        My = My[0] if isinstance(My, np.ndarray) and My.size == 1 else My
        Mz = Mz[0] if isinstance(Mz, np.ndarray) and Mz.size == 1 else Mz


        forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        ##### TODO #####
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = MAV.V_max * delta_t

        # Angular speed of propeller (omega_p = ?)
        a = (MAV.rho * MAV.D_prop**5)/((2*np.pi)**2) * MAV.C_Q0
        b = (MAV.rho * MAV.D_prop**4)/(2*np.pi) * MAV.C_Q1 * self._Va + MAV.KQ*MAV.KV/MAV.R_motor
        c = MAV.rho * MAV.D_prop**3 * MAV.C_Q2 * self._Va**2 - MAV.KQ*v_in/MAV.R_motor + MAV.KQ * MAV.i0
        omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)

        # thrust and torque due to propeller
        J = (2*np.pi * self._Va)/(omega_p * MAV.D_prop)
        CT = MAV.C_T2 * J**2 + MAV.C_T1 * J + MAV.C_T0
        CQ = MAV.C_Q2 * J**2 + MAV.C_Q1 * J + MAV.C_Q0

        thrust_prop = MAV.rho * (omega_p/(2*np.pi))**2 * MAV.D_prop**4 * CT
        torque_prop = MAV.rho * (omega_p/(2*np.pi))**2 * MAV.D_prop**5 * CQ

        return thrust_prop, torque_prop

    def _update_true_state(self):
        # rewrite this function because we now have more information
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0
