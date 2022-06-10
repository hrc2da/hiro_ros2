import numpy as np

class PIDControl:
    def __init__(self, p, i, d, initialized=False):
        self.err = 0
        self.last_obs = None
        self.target = 0
        self.integral = 0
        self.initialized = initialized
        self.kP=p
        self.kI=i
        self.kD=d

    def initialize(self,initial_target):
        self.target = initial_target
        self.initialized = True

    def setTarget(self, newTarget):
        if newTarget != self.target:
            self.target = newTarget
            #self.integral = 0

    def stepOutput(self, cur_val):
        # Diff current and target values (error)
        error = self.target - cur_val
        if self.last_obs is None:
            measurement_d = 0
        else:
            measurement_d = cur_val-self.last_obs

        # PID (wikipedia.org/wiki/PID_Controller)
                 # Absolute position error - approach faster if we're further
                 # away
        output = (self.kP * error
                 # Integral is the cumulative error - area under the error graph
                 + self.kI * self.integral
                 # Differential is error 'slope', diff between current and last
                 # error values
                #  + self.kD * (error - self.err))
                - measurement_d)

        # Store error for diff
        self.err=error
        self.last_obs = cur_val
        # Accumulate integral (remember, cumulative error)
        self.integral+=error

        return output
