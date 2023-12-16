
class PIDController:
    def __init__(self, setpoint, kp, ki, kd):
        self.setpoint = setpoint
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error, self.integral = 0, 0

    def calculate_output(self, current_value):
        error = self.setpoint - current_value
        self.integral = max(min(self.integral + error, 10), -10)
        P, I, D = self.kp * error, self.ki * self.integral, self.kd * (error - self.prev_error)
        output = P + I + D
        self.prev_error = error
        return output