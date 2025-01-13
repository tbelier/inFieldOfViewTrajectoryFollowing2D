import numpy as np
import matplotlib.pyplot as plt

def tanh(x):
    return np.tanh(x)

def tanh_derivative(x):
    return 1 - tanh(x)**2

def euler
def unicycle_dynamics(state, control_inputs, params, dt):
    """Compute the next state of the unicycle model."""
    s1, y1, theta, omega, v, s = state
    u1, u2 = control_inputs

    c1, c2, cc, gc = params['c1'], params['c2'], params['cc'], params['gc']

    # Compute derivatives
    ds1 = -s * (1 - cc * y1) + v * np.cos(theta)
    dy1 = -cc * s * s1 + v * np.sin(theta)
    dtheta = omega
    domega = u1 / c1 - cc * (v * np.cos(theta) + params['k1'] * s1) - gc * s**2
    dv = u2 / c2
    ds = v * np.cos(theta) + params['k1'] * s1

    # Update state using Euler integration
    s1_new = s1 + ds1 * dt
    y1_new = y1 + dy1 * dt
    theta_new = theta + dtheta * dt
    omega_new = omega + domega * dt
    v_new = v + dv * dt
    s_new = s + ds * dt

    return np.array([s1_new, y1_new, theta_new, omega_new, v_new, s_new])

def control_law(state, desired_state, params):
    """Compute the control inputs u1 and u2 based on the control law."""
    s1, y1, theta, omega, v, s = state
    vd = desired_state[0]

    # Gains and parameters
    theta_a, k_delta, k1, k2, k3, k4 = (
        params['theta_a'], params['k_delta'], params['k1'],
        params['k2'], params['k3'], params['k4']
    )

    # Compute delta (approach angle)
    delta = -theta_a * tanh(k_delta * y1 * v)

    # Derivatives of delta
    delta_y = -theta_a * k_delta * v * tanh_derivative(k_delta * y1 * v)
    delta_v = -theta_a * k_delta * y1 * tanh_derivative(k_delta * y1 * v)

    # Control law for u1 and u2
    u1 = params['c1'] * (-k3 * (theta - delta) - delta_y * y1 + k2 * (theta - delta))
    u2 = params['c2'] * (vd - k4 * (v - vd))

    return np.array([u1, u2])

# Simulation parameters
params = {
    'c1': 0.5,  # Moment of inertia-related parameter
    'c2': 1.0,  # Mass-related parameter
    'cc': 0.1,  # Coupling parameter
    'gc': 0.05,  # Gravity-related parameter
    'k1': 1.0,  # Gain for virtual target
    'k2': 2.0,  # Gain for angular control
    'k3': 1.5,  # Gain for angular velocity control
    'k4': 1.0,  # Gain for velocity control
    'theta_a': np.pi / 4,  # Maximum approach angle
    'k_delta': 1.0,  # Gain for delta function
}

# Initial state [s1, y1, theta, omega, v, s]
state = np.array([4.0, 30.0, -1.7, 0.0, 0.1, 10.0])

# Desired state [vd] (only desired velocity specified)
desired_state = np.array([1.0])

# Simulation settings
dt = 0.01  # Time step (s)
t_final = 10.0  # Final time (s)
n_steps = int(t_final / dt)

# Storage for simulation
states = np.zeros((n_steps, len(state)))
states[0] = state

# Run simulation
for i in range(1, n_steps):
    # Compute control inputs
    control_inputs = control_law(state, desired_state, params)

    # Update state
    state = unicycle_dynamics(state, control_inputs, params, dt)

    # Store state
    states[i] = state

# Plot results
plt.figure(figsize=(12, 8))

# Plot y1 vs s1 (trajectory)
plt.subplot(2, 2, 1)
plt.plot(states[:, 5], states[:, 1], label="Trajectory")
plt.xlabel("s (Virtual target position)")
plt.ylabel("y1 (Lateral error)")
plt.title("Lateral Error vs Virtual Target Position")
plt.grid()
plt.legend()

# Plot theta (orientation)
plt.subplot(2, 2, 2)
plt.plot(np.linspace(0, t_final, n_steps), states[:, 2], label="Theta")
plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.title("Orientation Over Time")
plt.grid()
plt.legend()

# Plot velocity
plt.subplot(2, 2, 3)
plt.plot(np.linspace(0, t_final, n_steps), states[:, 4], label="Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Over Time")
plt.grid()
plt.legend()

# Plot angular velocity
plt.subplot(2, 2, 4)
plt.plot(np.linspace(0, t_final, n_steps), states[:, 3], label="Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity Over Time")
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
