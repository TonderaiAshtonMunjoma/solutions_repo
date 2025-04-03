# Problem 2
# Investigating the Dynamics of a Forced Damped Pendulum

## 1. Theoretical Foundation

The forced damped pendulum is governed by a nonlinear differential equation that accounts for gravitational restoring force, damping, and external periodic forcing. Let’s derive the equation and explore its solutions.

### Governing Equation
For a pendulum of length $L$ with mass $m$ at angle $\theta$ from the vertical, the forces include:
- Gravitational torque: $-mgL \sin\theta$
- Damping torque: $-b \dot{\theta}$ (proportional to angular velocity, $b$ is the damping coefficient)
- External forcing torque: $F_0 \cos(\omega t)$ (amplitude $F_0$, frequency $\omega$)

The equation of motion, from Newton’s second law for rotation ($I \ddot{\theta} = \sum \tau$), is:
$$I \ddot{\theta} = -mgL \sin\theta - b \dot{\theta} + F_0 \cos(\omega t)$$
where $I = mL^2$ (moment of inertia). Dividing through by $I$:
$$\ddot{\theta} + \frac{b}{mL^2} \dot{\theta} + \frac{g}{L} \sin\theta = \frac{F_0}{mL^2} \cos(\omega t)$$
Define $\gamma = \frac{b}{mL^2}$ (damping rate), $\omega_0 = \sqrt{\frac{g}{L}}$ (natural frequency), and $f = \frac{F_0}{mL^2}$ (forcing amplitude per unit inertia). The equation becomes:
$$\ddot{\theta} + \gamma \dot{\theta} + \omega_0^2 \sin\theta = f \cos(\omega t)$$

### Small-Angle Approximation
For small $\theta$, $\sin\theta \approx \theta$, yielding a linear equation:
$$\ddot{\theta} + \gamma \dot{\theta} + \omega_0^2 \theta = f \cos(\omega t)$$
This is a driven damped harmonic oscillator. The steady-state solution is:
$$\theta(t) = A \cos(\omega t - \phi)$$
where amplitude $A = \frac{f}{\sqrt{(\omega_0^2 - \omega^2)^2 + (\gamma \omega)^2}}$ and phase $\phi = \tan^{-1}\left(\frac{\gamma \omega}{\omega_0^2 - \omega^2}\right)$.

### Resonance
Resonance occurs when $\omega \approx \omega_0$, maximizing $A$ if damping is low. The peak amplitude shifts slightly due to $\gamma$.

## 2. Analysis of Dynamics

### Parameter Influence
- **Damping ($\gamma$):** Low $\gamma$ allows sustained oscillations; high $\gamma$ suppresses motion.
- **Driving Amplitude ($f$):** Larger $f$ increases amplitude and can push the system into nonlinearity or chaos.
- **Driving Frequency ($\omega$):** Near $\omega_0$, resonance amplifies motion; far from $\omega_0$, motion may become quasiperiodic or chaotic.

### Transition to Chaos
For large $f$ or specific $\omega$, the nonlinear $\sin\theta$ term dominates, leading to:
- **Regular Motion:** Small $f$, near-linear oscillations.
- **Chaotic Motion:** High $f$, sensitive dependence on initial conditions, aperiodic behavior.

## 3. Practical Applications
- **Energy Harvesting:** Oscillatory motion converts to electrical energy (e.g., piezoelectric devices).
- **Suspension Bridges:** External forces (wind) can induce resonance or chaotic vibrations.
- **Circuits:** Driven RLC circuits mirror this behavior, used in signal processing.

## 4. Implementation

Below is a Python script to simulate the pendulum using the Runge-Kutta 4th-order (RK4) method and save plots for embedding.

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Pendulum parameters
g = 9.81  # m/s^2
L = 1.0   # m
omega0 = np.sqrt(g / L)
gamma = 0.5  # damping coefficient
f = 1.2      # driving amplitude
omega = 1.4 * omega0  # driving frequency

# Differential equation
def pendulum_deriv(state, t, gamma, omega0, f, omega):
    theta, theta_dot = state
    dtheta_dt = theta_dot
    dtheta_dot_dt = -gamma * theta_dot - omega0**2 * np.sin(theta) + f * np.cos(omega * t)
    return [dtheta_dt, dtheta_dot_dt]

# Time array
t = np.linspace(0, 50, 1000)

# Initial conditions
state0 = [0.1, 0.0]  # [theta, theta_dot]

# Solve ODE
sol = odeint(pendulum_deriv, state0, t, args=(gamma, omega0, f, omega))
theta, theta_dot = sol.T

# Plot 1: Time series
plt.figure(figsize=(10, 6))
plt.plot(t, theta, label=r'$\theta(t)$')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Forced Damped Pendulum Motion')
plt.legend()
plt.grid(True)
plt.savefig('pendulum_timeseries.png')
plt.close()

# Plot 2: Phase portrait
plt.figure(figsize=(10, 6))
plt.plot(theta, theta_dot, label='Phase Trajectory')
plt.xlabel(r'$\theta$ (rad)')
plt.ylabel(r'$\dot{\theta}$ (rad/s)')
plt.title('Phase Portrait')
plt.legend()
plt.grid(True)
plt.savefig('pendulum_phase.png')
plt.close()

# Plot 3: Poincaré section (at each driving period)
period = 2 * np.pi / omega
poincare_theta = []
poincare_theta_dot = []
for i in range(len(t)):
    if abs(t[i] % period) < 0.01:  # Sample at t = nT
        poincare_theta.append(theta[i])
        poincare_theta_dot.append(theta_dot[i])

plt.figure(figsize=(10, 6))
plt.scatter(poincare_theta, poincare_theta_dot, s=5, label='Poincaré Section')
plt.xlabel(r'$\theta$ (rad)')
plt.ylabel(r'$\dot{\theta}$ (rad/s)')
plt.title('Poincaré Section')
plt.legend()
plt.grid(True)
plt.savefig('pendulum_poincare.png')
plt.close()