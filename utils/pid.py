import matplotlib.pyplot as plt

class PID:
    def __init__(self, setpoint: float=None, Kp: float=None, Kd: float=None, Ki: float=None, dt: float=None):
        self.error_ = 0
        self.previous_error_ = 0

        self.dt_ = dt

        self.Kp_ = Kp
        self.Kd_ = Kd
        self.Ki_ = Ki

        self.setpoint_ = setpoint

        self.error_history_ = []

    def Propogate(self, current_output: float):
        # calculate error
        error = self.setpoint_ - current_output
        self.error_history_.append(error)

        output = 0

        # Scale the error with Kp
        if self.Kp_ is not None:
            error_proportional = self.Kp_ * error
            output += error_proportional

        # Calculate the derivative and scale it with Kd
        if self.Kd_ is not None:
            dedt = (error - self.previous_error_) / self.dt_
            output += dedt * self.Kd_
            self.previous_error_ = error

        # Integrate the error and scale it with Ki
        if self.Ki_ is not None:
            self.error_ += error
            output += self.error_ * self.Ki_

        return output
    
    def PlotPIDVars(self, timespan: list):
        plt.figure()

        plt.plot(timespan, self.error_history_)

        plt.title("PID error")
        plt.xlabel("t")
        plt.ylabel("error")