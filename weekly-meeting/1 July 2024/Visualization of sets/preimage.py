import numpy as np
import matplotlib.pyplot as plt

# Define the function h(x)
def h(x):
    return x**2 - 1

# Preimage of 0
preimage_0 = np.array([-1, 1])

# Small c close to 0
c = 0.1

# Preimage of c
preimage_c = np.array([np.sqrt(1 + c), -np.sqrt(1 + c)])

# Radius r2
r2 = 0.5

# Plotting
fig, ax = plt.subplots()

# Plot the preimage of 0
ax.plot(preimage_0, h(preimage_0), 'bo', label='$h^{-1}(0)$')

# Plot the preimage of c
ax.plot(preimage_c, h(preimage_c), 'ro', label='$h^{-1}(c)$')

# Minkowski sum visualization (balls around the preimage of 0)
for x in preimage_0:
    circle = plt.Circle((x, 0), r2, color='blue', fill=False, linestyle='--', label='$h^{-1}(0) \\oplus B_{r_2}$' if x == preimage_0[0] else "")
    ax.add_patch(circle)

# Settings
ax.set_xlabel('x')
ax.set_ylabel('h(x)')
ax.axhline(0, color='grey', lw=0.5)
ax.axvline(0, color='grey', lw=0.5)
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(-2, 2)
ax.set_ylim(-1.5, 1)
ax.legend()
plt.title('Visualization of Preimages and Upper Semi-Continuity')
plt.grid(True)
plt.show()
