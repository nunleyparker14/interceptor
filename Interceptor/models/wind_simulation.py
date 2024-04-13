"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from tools.transfer_function import TransferFunction
import numpy as np


class WindSimulation:
    def __init__(self, Ts, gust_flag = True, steady_state = np.array([[0., 0., 0.]]).T):
        # steady state wind defined in the inertial frame
        self._steady_state = steady_state
        ##### TODO #####

        # Dryden gust model parameters (pg 56 UAV book)

        # Get user input for scenario (1-4)
        scenario = 1

        # Initialize variables
        altitude = 0
        L_u = 0
        L_w = 0
        sig_u = 0
        sig_w = 0

        # Set variables based on the scenario
        if scenario == 1:
            altitude = 50
            L_u = L_v = 200
            L_w = 50
            sig_u = sig_v = 1.06
            sig_w = 0.7
        elif scenario == 2:
            altitude = 50
            L_u = L_v = 200
            L_w = 50
            sig_u = sig_v = 2.12
            sig_w = 1.4
        elif scenario == 3:
            altitude = 200
            L_u = L_v = 533
            L_w = 533
            sig_u = sig_v = 1.5
            sig_w = 1.5
        elif scenario == 4:
            altitude = 200
            L_u = L_v = 533
            L_w = 533
            sig_u = sig_v = 3.0
            sig_w = 3.0
        else:
            print("Invalid scenario. Please enter a scenario between 1 and 4.")
            exit()


        # pull in air speed from the parameter file, or hard  code it
        V_a = 25

        val_u = sig_u * np.sqrt((2*V_a)/(np.pi*L_u))
        val_v = sig_v * np.sqrt((2*V_a)/(np.pi*L_v))
        val_w = sig_w * np.sqrt((2*V_a)/(np.pi*L_w))

        H_u_num = np.array([[1/val_u]])
        H_u_den = np.array([[1/val_u, (V_a/L_u)/val_u]])

        H_v_num = np.array([[1/val_v, V_a/(np.sqrt(3*L_v))*val_v]])
        H_v_den = np.array([[1/val_v, (2*V_a)/(L_v*val_v),1/(L_v**2*val_v)]])

        H_w_num = np.array([[1/val_w, V_a/(val_w*np.sqrt(3*L_w))]])
        H_w_den = np.array([[1/val_w, (2*V_a)/(L_v*val_w),1/(L_v**2*val_w)]])

        # Dryden transfer functions (section 4.4 UAV book) - Fill in proper num and den
        self.u_w = TransferFunction(num=H_u_num, den=H_u_den,Ts=Ts)
        self.v_w = TransferFunction(num=H_v_num, den=H_v_den,Ts=Ts)
        self.w_w = TransferFunction(num=H_w_num, den=H_w_den,Ts=Ts)
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame

        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        
        
        return np.concatenate(( self._steady_state, gust ))

