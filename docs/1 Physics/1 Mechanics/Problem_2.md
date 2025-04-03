# Problem 2
# Investigating the Dynamics of a Forced Damped Pendulum

## Motivation

The forced damped pendulum is a fascinating physical system that exhibits complex behavior due to the interplay of damping, gravitational restoring forces, and external periodic forcing. Unlike a simple pendulum, the addition of damping and forcing introduces a range of dynamics—from simple harmonic motion to resonance, quasiperiodicity, and chaos. This system serves as a model for understanding real-world phenomena, such as mechanical oscillators, climate cycles, and structural vibrations under periodic loads. By exploring how parameters like damping, driving amplitude, and frequency affect the pendulum's motion, we gain insights into both fundamental physics and practical engineering applications, including energy harvesting and vibration control.

## Task Breakdown

### 1. Theoretical Foundation

The motion of a forced damped pendulum is governed by the following nonlinear differential equation:

$$
\frac{d^2\theta}{dt^2} + b \frac{d\theta}{dt} + \frac{g}{L} \sin(\theta) = F \cos(\omega t)
$$

Where:
- $\theta$: Angular displacement (radians)
- $b$: Damping coefficient (s⁻¹)
- $g$: Gravitational acceleration (m/s²)
- $L$: Pendulum length (m)
- $F$: Driving force amplitude (s⁻²)
- $\omega$: Driving frequency (rad/s)
- $t$: Time (s)

#### Small-Angle Approximation
For small angles, $\sin(\theta) \approx \theta$, simplifying the equation to a linear form:

$$
\frac{d^2\theta}{dt^2} + b \frac{d\theta}{dt} + \omega_0^2 \theta = F \cos(\omega t)
$$

Where $\omega_0 = \sqrt{\frac{g}{L}}$ is the natural frequency. This is a second-order linear differential equation resembling a driven damped harmonic oscillator. The general solution consists of a homogeneous solution (transient) and a particular solution (steady-state):

- **Homogeneous Solution**: $\theta_h(t) = A e^{-\frac{b}{2}t} \cos(\omega_d t + \phi)$, where $\omega_d = \sqrt{\omega_0^2 - \left(\frac{b}{2}\right)^2}$ is the damped frequency.
- **Particular Solution**: $\theta_p(t) = C \cos(\omega t) + D \sin(\omega t)$, where $C$ and $D$ are determined by substituting into the equation.

The steady-state amplitude is:

$$
A = \frac{F}{\sqrt{(\omega_0^2 - \omega^2)^2 + (b\omega)^2}}
$$

#### Resonance
Resonance occurs when the driving frequency $\omega$ approaches the natural frequency $\omega_0$, maximizing the amplitude. For light damping ($b \ll \omega_0$), the peak amplitude occurs near $\omega \approx \omega_0$, with energy transfer optimized.

### 2. Analysis of Dynamics

- **Damping Coefficient ($b$)**: Higher $b$ reduces amplitude and prevents unbounded growth at resonance, stabilizing the system but suppressing oscillations over time.
- **Driving Amplitude ($F$)**: Larger $F$ increases the steady-state amplitude and can push the system into nonlinear regimes, where $\sin(\theta) \neq \theta$ becomes significant.
- **Driving Frequency ($\omega$)**: Near $\omega_0$, resonance amplifies motion. Far from $\omega_0$, the system may exhibit quasiperiodic or chaotic behavior, especially with strong forcing.

The transition to chaos occurs in the nonlinear regime, where small changes in initial conditions or parameters lead to unpredictable motion. This is evident in phase portraits and Poincaré sections.

### 3. Practical Applications

- **Energy Harvesting**: Oscillatory motion in a forced pendulum can be converted to electrical energy via electromagnetic induction, as in wave energy devices.
- **Suspension Bridges**: Understanding resonance and damping helps design structures resistant to wind-induced oscillations (e.g., Tacoma Narrows Bridge collapse).
- **Oscillating Circuits**: The model parallels driven RLC circuits, informing electrical engineering designs.

### 4. Implementation

Below is a Python script using the Runge-Kutta method (RK4) to simulate the pendulum and visualize its dynamics.

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Parameters
g = 9.81  # gravity (m/s^2)
L = 1.0   # length (m)
b = 0.2   # damping coefficient (s^-1)
F = 1.2   # driving amplitude (s^-2)
omega = 2.0  # driving frequency (rad/s)
omega_0 = np.sqrt(g / L)  # natural frequency

# Differential equation
def pendulum_eq(y, t, b, omega_0, F, omega):
    theta, theta_dot = y
    dtheta_dt = theta_dot
    dtheta_dot_dt = -b * theta_dot - omega_0**2 * np.sin(theta) + F * np.cos(omega * t)
    return [dtheta_dt, dtheta_dot_dt]

# Time array
t = np.linspace(0, 50, 1000)

# Initial conditions
y0 = [0.1, 0.0]  # [theta, theta_dot]

# Solve ODE
sol = odeint(pendulum_eq, y0, t, args=(b, omega_0, F, omega))
theta, theta_dot = sol.T

# Phase portrait
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(t, theta, label="Angular Displacement")
plt.xlabel("Time (s)")
plt.ylabel("θ (rad)")
plt.title("Forced Damped Pendulum Motion")
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(theta, theta_dot, label="Phase Portrait")
plt.xlabel("θ (rad)")
plt.ylabel("dθ/dt (rad/s)")
plt.title("Phase Space")
plt.grid()
plt.legend()

plt.tight_layout()

# Poincaré section
poincare_theta = []
poincare_theta_dot = []
period = 2 * np.pi / omega
for i in range(len(t)):
    if abs(t[i] % period) < 0.01:  # Sample at driving period
        poincare_theta.append(theta[i])
        poincare_theta_dot.append(theta_dot[i])

plt.figure(figsize=(6, 6))
plt.scatter(poincare_theta, poincare_theta_dot, s=5, c='red', label="Poincaré Section")
plt.xlabel("θ (rad)")
plt.ylabel("dθ/dt (rad/s)")
plt.title("Poincaré Section")
plt.grid()
plt.legend()
plt.show()