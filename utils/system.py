import matplotlib.pyplot as plt
from utils.pid import PID


class System:
    def __init__(
        self,
        R: float = None,
        L: float = None,
        K: float = None,
        b: float = None,
        J: float = None,
        setpoint: float = None,
        Kp: float = None,
        Kd: float = None,
        Ki: float = None,
        dt: float = None,
    ):
        self.R_ = R
        self.L_ = L
        self.K_ = K
        self.b_ = b
        self.J_ = J
        self.dt_ = dt

        self.w0_ = 0
        self.i0_ = 0

        self.w_history_ = []

        self.pid_controller_ = PID(setpoint, Kp, Kd, Ki, dt)

    def Model(self, u: float, dtheta: float, i: float):
        # calculate ddx and dx
        b = self.b_
        J = self.J_
        K = self.K_
        R = self.R_
        L = self.L_

        ddtheta = (-b / J) * dtheta + (K / J) * i
        di = (-K / L) * dtheta - (R / L) * i + u / L

        return ddtheta, di

    def Simulate(self, timespan: list):
        w_t = self.w0_
        i_t = self.i0_

        for t in timespan:
            # apply PID controller
            u = self.pid_controller_.Propogate(w_t)
            ddtheta, di = self.Model(u, w_t, i_t)

            self.w_history_.append(w_t)

            w_t_plus_1 = w_t + ddtheta * self.dt_
            i_t_plus_1 = i_t + di * self.dt_

            w_t = w_t_plus_1
            i_t = i_t_plus_1

        print(f"Simulation finished with history size of: {len(self.w_history_)}")

    def PlotSystemVars(self, timespan: list):
        plt.figure()

        plt.plot(timespan, self.w_history_)

        plt.title("Angular Velocity Graph")
        plt.xlabel("t")
        plt.ylabel("w")

        plt.axhline(self.pid_controller_.setpoint_)

        self.pid_controller_.PlotPIDVars(timespan)

        plt.show()
