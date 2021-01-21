import numpy as np
from sim.sim2d_self import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]

options['DRIVE_IN_CIRCLE'] = False
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options['MEASURE_ANGLE'] = False
options['RECIEVE_INPUTS'] = False

class KalmanFilter:
    def __init__(self):
        self.v = 0
        self.prev_time = 0
        # Initial State defined as [x, y, x_dot, y_dot]
        self.x = np.matrix([[0.],
                            [0.],
                            [0.],
                            [0]])

        # Uncertainity Matrix
        self.P = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.],])

        # Next State Function
        self.F = np.matrix([[1., 0., 1., 0.],
                            [0., 1., 0., 1.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.],])

        # Measurement Function
        self.H = np.matrix([[1., 0., 0., 0.],
                            [0., 0., 1., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[1., 0.],
                            [0., 1.]])

        # Identity Matrix
        self.I = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])
    def predict(self, t):
        dt = t - self.prev_time
        self.F[0,2] = dt
        self.F[1,3] = dt
        self.x = self.F * self.x  # Update state
        self.P =  self.F*self.P  * np.transpose(self.F) # Update state uncertainity 
        return self.x[0,0],self.x[2,0]

        return
    def measure_and_update(self,measurements, t):
        dt = t - self.prev_time
        self.F[0,2] = dt
        self.F[2,3] = dt
        Z = np.matrix(measurements) # x_t being measured from sim1d
        y = np.transpose(Z) - self.H * self.x # y is the error between measurement and presdiction
        S = self.H * self.P * np.transpose(self.H) +self.R
        K  = self.P * np.transpose(self.H) * np.linalg.inv(S) # Kalman gain
        self.x = self.x + K * y # update state with kalman gain
        self.p = (self.I - K*self.H)*self.P # Update state uncertainity matrix based on the kalman gain calculated
        self.v = self.x[1,0] # Assign updated velocity or x_dot to plot the graph
        self.prev_time = t
        # return self.x, self.P


        return [self.x[0], self.x[2],self.P]

    def recieve_inputs(self, u_steer, u_pedal):
        u_pedal = 10
        u_steer = 10
        return

sim_run(options,KalmanFilter)
