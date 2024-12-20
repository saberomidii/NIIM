import matplotlib.pyplot as plt
import numpy as np

# Define radii
r1 = 1
r2 = 2
r3 = r1 + r2

# Define the circles
circle_c = plt.Circle((0, 0), r1, color='blue', fill=False, linestyle='--', label='Boundary of $\mathcal{C}$')
circle_c_delta = plt.Circle((0, 0), r3, color='green', fill=False, linestyle='-', label='Boundary of $\partial \mathcal{C} \oplus \overline{B}_{r_2}$')

# Define points
x3 = np.array([r3, 0])  # Point on the boundary of expanded safe set
x2 = np.array([r1, 0])  # Point on the boundary of original safe set within r2 of x3
x1 = np.array([0.5, 0.5])  # Point within the set D within r1 of x2

# Create the plot
fig, ax = plt.subplots()

# Add circles to plot
ax.add_patch(circle_c)
ax.add_patch(circle_c_delta)

# Plot points
ax.plot(*x3, 'ro', label='$\mathbf{x}_3$')
ax.plot(*x2, 'go', label='$\mathbf{x}_2$')
ax.plot(*x1, 'bo', label='$\mathbf{x}_1$')

# Draw lines showing distances
plt.plot([x3[0], x2[0]], [x3[1], x2[1]], 'k--', label='$\|\mathbf{x}_2 - \mathbf{x}_3\| \leq r_2$')
plt.plot([x2[0], x1[0]], [x2[1], x1[1]], 'k-.', label='$\|\mathbf{x}_1 - \mathbf{x}_2\| \leq r_1$')
plt.plot([x3[0], x1[0]], [x3[1], x1[1]], 'k:', label='$\|\mathbf{x}_1 - \mathbf{x}_3\| \leq r_3$')

# Set limits and labels
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Visualization of Proof in Theorem 2')

# Show the plot
plt.grid(True)
plt.show()
