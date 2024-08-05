import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from pid_controller import AdaptivePIDController
import math 

def conveyor_dynamics(y, t, pid_controller, max_speed=2.0, max_acceleration=0.1):
    position, velocity = y
    
    # Time-varying setpoint
    setpoint = 1.0 + 0.2 * math.sin(0.1 * t)
    
    # Time-varying disturbance
    disturbance = 0.1 * math.sin(0.5 * t)
    
    # PID control for motor speed
    desired_speed = pid_controller.compute(setpoint, position)
    
    # Add disturbance to desired speed
    desired_speed += disturbance
    
    # Constrain the desired speed by max speed
    desired_speed = max(min(desired_speed, max_speed), -max_speed)
    
    # Calculate acceleration needed to reach desired speed
    acceleration = (desired_speed - velocity) / pid_controller.dt
    
    # Time-varying friction
    friction = 0.05 + 0.02 * math.cos(0.3 * t)
    acceleration -= friction * velocity
    
    # Constrain the acceleration by max acceleration
    acceleration = max(min(acceleration, max_acceleration), -max_acceleration)
    
    dposition_dt = velocity
    dvelocity_dt = acceleration
    
    return [dposition_dt, dvelocity_dt]

# Initialize Adaptive PID controller
dt = 0.01
pid = AdaptivePIDController(dt=dt)

# Initial conditions
initial_conditions = [0.0, 0.0]  # [position, velocity]
time = np.linspace(0, 600, int(600 / dt))

# Solve the ODE
solution = odeint(conveyor_dynamics, initial_conditions, time, args=(pid,))

# Extract results
position, velocity = solution.T

# Plot results
plt.figure(figsize=(14, 12))
plt.subplot(3, 1, 1)
plt.plot(time, position, 'r-', label='Position')
plt.plot(time, 1.0 + 0.2 * np.sin(0.1 * time), 'b--', label='Setpoint')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Position Over Time with Adaptive PID Control')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, velocity, 'g-', label='Velocity')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity Over Time with Adaptive PID Control')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, [pid.kp for _ in time], 'r-', label='Kp')
plt.plot(time, [pid.ki for _ in time], 'g-', label='Ki')
plt.plot(time, [pid.kd for _ in time], 'b-', label='Kd')
plt.xlabel('Time')
plt.ylabel('PID Parameters')
plt.title('Adaptive PID Parameters Over Time')
plt.legend()

plt.tight_layout()
plt.savefig('adaptive_pid_simulation_results.png')
print("Simulation complete. Results saved to 'adaptive_pid_simulation_results.png'")