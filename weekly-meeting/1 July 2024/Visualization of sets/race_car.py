import plotly.graph_objects as go
import numpy as np

# Define the outer oval
outer_x1 = np.linspace(-6, 6, 400)
outer_y1 = np.sqrt(1 - (outer_x1 / 6) ** 2) + 1
outer_z1 = np.zeros_like(outer_x1)

outer_x2 = np.linspace(6, -6, 400)
outer_y2 = -np.sqrt(4 - (outer_x2 / 6) ** 2) + 1
outer_z2 = np.zeros_like(outer_x2)

# Define the inner oval
inner_x1 = np.linspace(-3, 3, 200)
inner_y1 = np.sqrt(4 - (inner_x1 / 3) ** 2) + 1
inner_z1 = np.zeros_like(inner_x1)

inner_x2 = np.linspace(3, -3, 200)
inner_y2 = -np.sqrt(1 - (inner_x2 / 3) ** 2) + 1
inner_z2 = np.zeros_like(inner_x2)



# Create the 3D plot
fig = go.Figure()

# Add the outer oval track with colors
fig.add_trace(go.Scatter3d(x=outer_x1, y=outer_y1, z=outer_z1, mode='lines', line=dict(color='blue', width=2), showlegend=False))
fig.add_trace(go.Scatter3d(x=outer_x2, y=outer_y2, z=outer_z2, mode='lines', line=dict(color='blue', width=2), showlegend=False))

# Add the inner oval track with colors
fig.add_trace(go.Scatter3d(x=inner_x1, y=inner_y1, z=inner_z1, mode='lines', line=dict(color='red', width=2), showlegend=False))
fig.add_trace(go.Scatter3d(x=inner_x2, y=inner_y2, z=inner_z2, mode='lines', line=dict(color='red', width=2), showlegend=False))



# Set limits and labels
fig.update_layout(
    scene=dict(
        xaxis=dict(title='X-axis', range=[-7, 7]),
        yaxis=dict(title='Y-axis', range=[-4, 4]),
        zaxis=dict(title='Z-axis', range=[-1, 1]),
        aspectratio=dict(x=2, y=1, z=0.2)
    ),
    title="3D Visualization of Expert Controller"
)

fig.show()
