import numpy as np

class AdaptivePIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limit = 10.0
        self.integral_limit = 2.0  # Reduced integral limit
        
        # Adaptation parameters
        self.kp_min, self.kp_max = 0.1, 3.0  # Further reduced max Kp
        self.ki_min, self.ki_max = 0.0, 0.5  # Reduced Ki range
        self.kd_min, self.kd_max = 0.0, 5.0  # Increased Kd range
        self.adaptation_rate = 0.001  # Slower adaptation rate
        
        # Error history for improved adaptation
        self.error_history = []
        self.error_history_max_length = 200

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value     #added the accurent error to the list of error history and check if the len of this list > than the max   
        self.error_history.append(error)
        if len(self.error_history) > self.error_history_max_length:
            self.error_history.pop(0)     #added the accurent error to the list of error history and check if the len of this list > than the max
        
        # Adapt PID parameters
        self.adapt_parameters()
        
        # Proportional term
        P = self.kp * error
        
        # Integral term with anti-windup and decay
        if -self.output_limit < P < self.output_limit:    #check if the P between the output limits ==> this can effect the I and caused "integral windup"
            self.integral += error * self.dt
        else:
            self.integral *= 0.9  # decrease the integral term by 10% when output saturated
              #when the controllers output is satureted,instead of allowing the integral term to keep growing,it is decayed 
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        I = self.ki * self.integral
        
        # Derivative term with filtering
        derivative = (error - self.previous_error) / self.dt
        D = self.kd * derivative
        
        # Calculate output with limiting
        output = P + I + D
        output = max(min(output, self.output_limit), -self.output_limit)
        
        self.previous_error = error
        return output

    def adapt_parameters(self):
        if len(self.error_history) < 10: # Start adpat_parameters if there are at least 10 errors recorded 
            return

        error_magnitude = abs(self.error_history[-1]) 
        error_change = abs(self.error_history[-1] - self.error_history[-2])
        error_sum = sum(map(abs, self.error_history[-20:]))  # Sum of last 20 errors

        # Adapt Kp based on error magnitude and trend

        error_trend = sum(np.diff(self.error_history[-10:])) # the sum of deiffereces between consecutives error over the last 10 errors

        if abs(error_trend) < 0.1 * sum(map(abs, self.error_history[-10:])):    #compar error_tend with the 10% of the sum of the last 10 errors 
            self.kp *= 1.05  # Increase Kp if error is not changing significantly
        else:
            self.kp *= 0.95  # Decrease Kp if error is changing

        # Adapt Ki based on long-term error (integral action)
        if abs(error_sum) > 5 * self.integral_limit:
            self.ki *= 1.05
        else:
            self.ki *= 0.99

        #The condition if abs(error_trend) < 0.1 * sum(map(abs, self.error_history[-10:])): is used to determine whether the recent error trend is stable or changing significantly. If the trend is stable (less than 10% of the cumulative recent errors), the method will increase Kp to enhance the controller's response. If the trend is changing significantly, Kp will be decreased to stabilize the system.

        # Adapt Kd based on error change (derivative action)
        if error_change > 0.1 * error_magnitude:
            self.kd *= 1.05
        else:
            self.kd *= 0.99

        # Ensure parameters are within limits
        self.kp = max(min(self.kp, self.kp_max), self.kp_min)
        self.ki = max(min(self.ki, self.ki_max), self.ki_min)
        self.kd = max(min(self.kd, self.kd_max), self.kd_min)

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.error_history.clear()