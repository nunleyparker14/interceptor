import numpy as np
from tools.rotations import euler_to_quaternion

class TargParam:
    def __init__(self):
        ######################################################################################
        #   Initial Conditions
        ######################################################################################
        #   Initial conditions for MAV
        self.north0 = -1000.  # initial north position
        self.east0 = -1000.  # initial east position
        self.down0 = -1000.0  # initial down position
        self.u0 = 25.  # initial velocity along body x-axis
        self.v0 = 0.  # initial velocity along body y-axis
        self.w0 = 0.  # initial velocity along body z-axis
        self.phi0 = 0.  # initial roll angle
        self.theta0 = 0.  # initial pitch angle
        self.psi0 = 0.0  # initial yaw angle
        self.p0 = 0  # initial roll rate
        self.q0 = 0  # initial pitch rate
        self.r0 = 0  # initial yaw rate
        self.Va0 = np.sqrt(self.u0**2 + self.v0**2 + self.w0**2)
        #   Quaternion State
        self.e = euler_to_quaternion(self.phi0, self.theta0, self.psi0)
        self.e0 = self.e.item(0)
        self.e1 = self.e.item(1)
        self.e2 = self.e.item(2)
        self.e3 = self.e.item(3)

        ######################################################################################
        #   Physical Parameters
        ######################################################################################
        self.mass = 11. #kg
        self.Jx = 0.8244 #kg m^2
        self.Jy = 1.135
        self.Jz = 1.759
        self.Jxz = 0.1204
        self.S_wing = 0.55
        self.b = 2.8956
        self.c = 0.18994
        self.S_prop = 0.2027
        self.rho = 1.2682
        self.e = 0.9
        self.AR = (self.b**2) / self.S_wing
        self.gravity = 9.81

        ######################################################################################
        #   Longitudinal Coefficients
        ######################################################################################
        self.C_L_0 = 0.23
        self.C_D_0 = 0.043
        self.C_m_0 = 0.0135
        self.C_L_alpha = 5.61
        self.C_D_alpha = 0.03
        self.C_m_alpha = -2.74
        self.C_L_q = 7.95
        self.C_D_q = 0.0
        self.C_m_q = -38.21
        self.C_L_delta_e = 0.13
        self.C_D_delta_e = 0.0135
        self.C_m_delta_e = -0.99
        self.M = 50.0
        self.alpha0 = 0.47
        self.epsilon = 0.16
        self.C_D_p = 0.0

        ######################################################################################
        #   Lateral Coefficients
        ######################################################################################
        self.C_Y_0 = 0.0
        self.C_ell_0 = 0.0
        self.C_n_0 = 0.0
        self.C_Y_beta = -0.98
        self.C_ell_beta = -0.13
        self.C_n_beta = 0.073
        self.C_Y_p = 0.0
        self.C_ell_p = -0.51
        self.C_n_p = 0.069
        self.C_Y_r = 0.0
        self.C_ell_r = 0.25
        self.C_n_r = -0.095
        self.C_Y_delta_a = 0.075
        self.C_ell_delta_a = 0.17
        self.C_n_delta_a = -0.011
        self.C_Y_delta_r = 0.19
        self.C_ell_delta_r = 0.0024
        self.C_n_delta_r = -0.069

        ######################################################################################
        #   Propeller thrust / torque parameters (see addendum by McLain)
        ######################################################################################
        # Prop parameters
        self.D_prop = 20*(0.0254)     # prop diameter in m

        # Motor parameters
        self.KV_rpm_per_volt = 145.                            # Motor speed constant from datasheet in RPM/V
        self.KV = (1. / self.KV_rpm_per_volt) * 60. / (2. * np.pi)  # Back-emf constant, KV in V-s/rad
        self.KQ = self.KV                                           # Motor torque constant, KQ in N-m/A
        self.R_motor = 0.042              # ohms
        self.i0 = 1.5                     # no-load (zero-torque) current (A)

        # Inputs
        self.ncells = 12.
        self.V_max = 3.7 * self.ncells  # max voltage for specified number of battery cells

        # Coeffiecients from prop_data fit
        self.C_Q2 = -0.01664
        self.C_Q1 = 0.004970
        self.C_Q0 = 0.005230
        self.C_T2 = -0.1079
        self.C_T1 = -0.06044
        self.C_T0 = 0.09357

        ######################################################################################
        #   Calculation Variables
        ######################################################################################
        #   gamma parameters pulled from page 36 (dynamics)
        self.gamma = self.Jx * self.Jz - (self.Jxz**2)
        self.gamma1 = (self.Jxz * (self.Jx - self.Jy + self.Jz)) / self.gamma
        self.gamma2 = (self.Jz * (self.Jz - self.Jy) + (self.Jxz**2)) / self.gamma
        self.gamma3 = self.Jz / self.gamma
        self.gamma4 = self.Jxz / self.gamma
        self.gamma5 = (self.Jz - self.Jx) / self.Jy
        self.gamma6 = self.Jxz / self.Jy
        self.gamma7 = ((self.Jx - self.Jy) * self.Jx + (self.Jxz**2)) / self.gamma
        self.gamma8 = self.Jx / self.gamma

        #   C values defines on pag 62
        self.C_p_0         = self.gamma3 * self.C_ell_0      + self.gamma4 * self.C_n_0
        self.C_p_beta      = self.gamma3 * self.C_ell_beta   + self.gamma4 * self.C_n_beta
        self.C_p_p         = self.gamma3 * self.C_ell_p      + self.gamma4 * self.C_n_p
        self.C_p_r         = self.gamma3 * self.C_ell_r      + self.gamma4 * self.C_n_r
        self.C_p_delta_a   = self.gamma3 * self.C_ell_delta_a + self.gamma4 * self.C_n_delta_a
        self.C_p_delta_r   = self.gamma3 * self.C_ell_delta_r + self.gamma4 * self.C_n_delta_r
        self.C_r_0         = self.gamma4 * self.C_ell_0      + self.gamma8 * self.C_n_0
        self.C_r_beta      = self.gamma4 * self.C_ell_beta   + self.gamma8 * self.C_n_beta
        self.C_r_p         = self.gamma4 * self.C_ell_p      + self.gamma8 * self.C_n_p
        self.C_r_r         = self.gamma4 * self.C_ell_r      + self.gamma8 * self.C_n_r
        self.C_r_delta_a   = self.gamma4 * self.C_ell_delta_a + self.gamma8 * self.C_n_delta_a
        self.C_r_delta_r   = self.gamma4 * self.C_ell_delta_r + self.gamma8 * self.C_n_delta_r
