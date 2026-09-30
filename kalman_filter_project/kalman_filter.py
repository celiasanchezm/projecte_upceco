"""
THIS is the file you need to complete.


A reasonable state to estimate is [x, y, phi, v], matching the simulator's
ground truth, with the IMU acceleration and yaw rate as the prediction
input and the GPS position as the measurement (see section 1.2 of the
project statement, "Vehicle model and linearization"). If that feels like
too much at once, start from a simpler model, for example position and
velocity along a straight line, before adding heading.
"""
import numpy as np


class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, Q, R, wheelbase):
        self.x = np.array(initial_state, dtype=float)
        self.P = np.array(initial_covariance, dtype=float) #uncertainty of the estimate
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.wheelbase = wheelbase

    def predict(self, u, dt):
        """Prediction step: propagate self.x and self.P using the motion
        model, given the input u = (accel, yaw_rate) from the IMU and the
        time step dt (called once per IMU sample from main.py).
        """
        
        x, y, phi, v = self.x 

        accel, yaw_rate = u
        #calculations for where i believe the car is now using my last state + what IMU just mesured
        x_new = x + v * np.cos(phi) * dt  #Xnew = Xold + speed*time 
        y_new = y + v * np.sin(phi) * dt   
        phi_new = phi + yaw_rate * dt   #ºnew = ºold + yaw rate*time
        v_new = v + accel * dt  #Vnew = Vold + acc*time

        self.x = np.array([x_new, y_new, phi_new, v_new]) #assign the new variables to the state vector

        #F is the matrix for propagation of the covariance when doing an estimation
        F = np.array([
            [1, 0, -v * np.sin(phi) * dt, np.cos(phi) * dt],
            [0, 1,  v * np.cos(phi) * dt, np.sin(phi) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.P = F @ self.P @ F.T + self.Q #we propagate the last P thru F and sum the covariance of the process
        

    def update_gps(self, z):
        """Update step: correct self.x and self.P using a GPS position """
        raise NotImplementedError(
            "Implement the update step here (see sections 1.2 and 1.3 "
            "of the project statement and the Estimation training script)."
        )
