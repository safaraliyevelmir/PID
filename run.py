import utils.system as sys
from utils import pid, pid_controller as pidController, dc_motor as dcMotor
import numpy as np
import matplotlib.pyplot as plt

# System Parameters
J, K, R, L = 0.01, 0.01, 1, 0.5
b, w_setpoint, Kp, Ki, Kd = 0.1, 10.0, 0.5, 0.1, 0.01
simulation_time, dt = 30, 0.01
timespan = np.arange(0, simulation_time, dt)

# System Simulation
mass_spring_damper = sys.System(R, L, K, b, J, w_setpoint, Kp, Kd, Ki, dt)
mass_spring_damper.Simulate(timespan)
mass_spring_damper.PlotSystemVars(timespan)

pid_object = pid.PID(w_setpoint, Kp, Kd, Ki, dt)
pid_object.Propogate(timespan)
pid_object.PlotPIDVars(timespan)

def run_simulation(dc_motor, pid_controller, initial_value, time_step, simulation_steps):
    time_values = []
    process_values = []

    current_value = initial_value
    for step in range(simulation_steps):
        time_values.append(step * time_step)
        process_values.append(current_value)

        control_signal = pid_controller.calculate_output(current_value)
        current_value = dc_motor.update(control_signal, time_step)

    return time_values, process_values

def plot_results(time_values, process_values, setpoint):
    fig, ax = plt.subplots()
    ax.plot(time_values, process_values, label="Process Value", color="blue")
    ax.axhline(y=setpoint, color="r", linestyle="--", label="Setpoint")

    ax.set_facecolor("black")
    ax.set_title("PID Controller with DC Motor Simulation")
    ax.set_xlabel("Time Steps")
    ax.set_ylabel("Process Value")
    ax.legend()
    ax.grid(True)
    plt.show()

# DC motor and PID Controller Parameters
motor_params = {"k": 1.0, "J": 0.01, "b": 0.1}
setpoint, kp, ki, kd = 10.0, 0.5, 0.1, 0.01

# Initialize Classes
dc_motor = dcMotor.DCMotor(**motor_params)
pid_controller = pidController.PIDController(setpoint, kp, ki, kd)

# Simulation Parameters
initial_value, time_step, simulation_steps = 3.0, 0.1, 100

# Run Simulation
time_values, process_values = run_simulation(dc_motor, pid_controller, initial_value, time_step, simulation_steps)

# Plot Results
plot_results(time_values, process_values, setpoint)
