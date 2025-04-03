# Problem 1
# Investigating the Range as a Function of the Angle of Projection

## 1. Theoretical Foundation

Projectile motion describes the path of an object under the influence of gravity, neglecting air resistance. Let’s derive the governing equations from Newton’s second law.

### Derivation of Equations
Consider a projectile launched with initial velocity $v_0$ at an angle $\theta$ from the horizontal. The acceleration due to gravity is $g$ (downward). We break the motion into horizontal (x) and vertical (y) components:

- **Initial conditions:**
  - Horizontal velocity: $v_{x0} = v_0 \cos\theta$
  - Vertical velocity: $v_{y0} = v_0 \sin\theta$
  - Initial position: $(x_0, y_0) = (0, 0)$ (assuming launch from origin)

- **Acceleration:**
  - $a_x = 0$ (no horizontal forces)
  - $a_y = -g$ (gravity acts downward)

The differential equations are:
- Horizontal: $\frac{d^2x}{dt^2} = 0$
- Vertical: $\frac{d^2y}{dt^2} = -g$

Integrating with respect to time:
- Horizontal: $v_x = v_{x0} = v_0 \cos\theta$, then $x = v_0 \cos\theta \cdot Linda t$
- Vertical: $v_y = v_{y0} - g t = v_0 \sin\theta - g t$, then $y = v_0 \sin\theta \cdot t - \frac{1}{2} g t^2$

### Time of Flight
The projectile hits the ground when $y = 0$:
$$0 = v_0 \sin\theta \cdot t - \frac{1}{2} g t^2$$
Factorizing: $t (v_0 \sin\theta - \frac{1}{2} g t) = 0$
Solutions: $t = 0$ (launch) or $t = \frac{2 v_0 \sin\theta}{g}$ (landing). Thus, time of flight is:
$$T = \frac{2 v_0 \sin\theta}{g}$$

### Family of Solutions
The position $(x, y)$ depends on $t$, $v_0$, $\theta$, and $g$. Varying these parameters generates a family of parabolic trajectories.

## 2. Analysis of the Range

### Range Equation
The horizontal range $R$ is the x-distance when $y = 0$:
$$R = v_0 \cos\theta \cdot T = v_0 \cos\theta \cdot \frac{2 v_0 \sin\theta}{g}$$
Using the identity $2 \sin\theta \cos\theta = \sin(2\theta)$:
$$R = \frac{v_0^2 \sin(2\theta)}{g}$$

### Dependence on Angle
- $R$ is maximized when $\sin(2\theta) = 1$, i.e., $2\theta = 90^\circ$, so $\theta = 45^\circ$.
- $R = 0$ at $\theta = 0^\circ$ and $\theta = 90^\circ$ (since $\sin(0) = \sin(180^\circ) = 0$).
- Range is symmetric about $45^\circ$ (e.g., $R(\theta) = R(90^\circ - \theta)$).

### Parameter Influence
- **Initial Velocity ($v_0$):** $R \propto v_0^2$, a quadratic relationship.
- **Gravity ($g$):** $R \propto \frac{1}{g}$, inversely proportional.

## 3. Practical Applications

- **Sports:** Optimizing a basketball shot or a soccer kick involves finding the ideal $\theta$ for distance and height.
- **Engineering:** Artillery or rocket launches adjust $\theta$ and $v_0$ based on target range.
- **Uneven Terrain:** If launched from height $h$, the range equation modifies (requires quadratic solution).
- **Air Resistance:** Introduces a damping term, reducing $R$ and requiring numerical solutions.

## 4. Implementation

Below is a Python script to simulate and visualize the range vs. angle.

```python
import numpy as np
import matplotlib.pyplot as plt

def calculate_range(v0, theta_deg, g=9.81):
    """Calculate range given initial velocity, angle (degrees), and gravity."""
    theta = np.radians(theta_deg)
    return (v0**2 * np.sin(2 * theta)) / g

# Parameters
v0_values = [10, 20, 30]  # m/s
g = 9.81  # m/s^2
theta_deg = np.linspace(0, 90, 91)  # 0 to 90 degrees

# Compute ranges
plt.figure(figsize=(10, 6))
for v0 in v0_values:
    R = [calculate_range(v0, t, g) for t in theta_deg]
    plt.plot(theta_deg, R, label=f'v0 = {v0} m/s')

# Plot settings
plt.xlabel('Angle of Projection (degrees)')
plt.ylabel('Range (m)')
plt.title('Range vs. Angle of Projection')
plt.legend()
plt.grid(True)
plt.show()

