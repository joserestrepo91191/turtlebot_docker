"""Control PID simple"""

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        
        # for plotting
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        self.p_term = self.kp * error
        self.i_term = self.ki * self.integral
        self.d_term = self.kd * derivative   
             
        output = self.p_term + self.i_term + self.d_term
        self.prev_error = error
        
        return output