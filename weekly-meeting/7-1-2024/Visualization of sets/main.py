import plotly.graph_objects as go
import numpy as np

# Parameters
radius = 1
theta_range = np.linspace(0, np.pi/2, 100)  # Angle range from 0 to 90 degrees
z_height = 1  # Height of the 3D extrusion

# Create meshgrid for the sector in the x-y plane
theta, r = np.meshgrid(theta_range, np.linspace(0, radius, 50))
x = r * np.cos(theta)
y = r * np.sin(theta)

# Define the quadratic function for z
theta_rod = np.pi / 4  # Example angle
z = -0.5*((theta - theta_rod) ** 2)

# Calculate distance from the rod for color scaling
distance_from_rod = np.abs(theta - theta_rod)

# Define custom colorscale from blue to red
colorscale = [[0, 'blue'], [1, 'red']]

# Extrude the sector in the z-direction
x_3d = np.vstack([x, x])
y_3d = np.vstack([y, y])
z_3d = np.vstack([z, z])
distance_3d = np.vstack([distance_from_rod, distance_from_rod])

# Create the 3D plot
fig = go.Figure(data=[go.Surface(x=x_3d, y=y_3d, z=z_3d, surfacecolor=distance_3d, colorscale=colorscale, opacity=1)])

# Add the red rod
rod_x = np.array([0, radius])
rod_y = np.array([0, 0])
rod_z = np.array([0, 0])

# Rotate rod by theta
rod_x_rotated = rod_x * np.cos(theta_rod) - rod_y * np.sin(theta_rod)
rod_y_rotated = rod_x * np.sin(theta_rod) + rod_y * np.cos(theta_rod)

fig.add_trace(go.Scatter3d(x=rod_x_rotated, y=rod_y_rotated, z=rod_z, mode='lines', line=dict(color='black', width=10)))

# Update layout for better visualization
fig.update_layout(
    scene=dict(
        xaxis=dict(title='X-axis'),
        yaxis=dict(title='Y-axis'),
        zaxis=dict(title='Z-axis'),
        aspectratio=dict(x=1, y=1, z=0.5)
    ),
    title="3D Visualization of the Sector with Color Scaling"
)

fig.show()
