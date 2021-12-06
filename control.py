#!/usr/bin/env python3

import rospy


class PI_controller(object):
    """PI controller."""

    def __init__(self, Kp=1.0, Ki=0, setpoint=0, sample_time=0.01, output_constraints=(None, None)):
        """PI controller.

            :param Kp: proportional gain, Kp
            :param Ki: integral gain, Ki
            :param setpoint: goal that controller is trying to attain
            :param sample_time: minimum time required before calculating again
            :param output_constraints: allowable lower and upper bounds on the output
        """

        self.Kp, self.Ki = Kp, Ki
        self.setpoint = setpoint

        # Set the sample time limit as a ROS duration
        self.sample_time = sample_time
        self.output_constraints = output_constraints

        self.proportional = 0
        self.integral     = 0

        self.last_input  = None
        self.last_output = None
        self.last_dt     = None


    def _constrain(self, val, bounds):
        """Constrain the value to within a boundary of values"""

        lower, upper = bounds

        if val is None: return None

        if (upper is not None) and (val > upper):
            return upper
        elif (lower is not None) and (val < lower):
            return lower

        return val


    def __call__(self, input, dt=None):
        """Update the PI controller"""

        now = rospy.Time.now().to_sec()

        # Set the time delta
        if dt is None:
            dt = now - self.last_dt if (self.last_dt is not None) else 1e-10
        elif dt <= 0:
            raise ValueError('dt is negative, only positve values allowed.')

        # Only return when enough time has elapsed, otherwise just give back the previous value
        if self.sample_time is not None and (dt < self.sample_time) and self.last_output is not None:
            return self.last_output

        # Calculate the error and the change in input value from the previous value
        # Max error is 580
        err = self.setpoint - input

        delta_input = input - (self.last_input if (self.last_input is not None) else input)

        # Calculate the proportional term value
        self.proportional = self.Kp*err

        # Calculate the integral term value and constrain to prevent runaway
        self.integral += self.Ki*err*dt
        self.integral  = self._constrain(self.integral, self.output_constraints)

        # Calculate the final output and constriain to prevent runaway
        output = self.proportional + self.integral
        # print(output)
        output = self._constrain(output, self.output_constraints)

        # Store values for next iteration
        self.last_input  = input
        self.last_output = output
        self.last_dt     = now

        return output
