import numpy as np
import matplotlib.pyplot as plt

# Define the range of x values
x = np.linspace(-10, 10, 400)

# Define different values of the constant C
C_values = [-5, 0, 5]

# Create the plot
plt.figure(figsize=(8, 6))
for C in C_values:
    y = (x**2) / 2 + C
    plt.plot(x, y, label=f'C = {C}')

# Add title and labels
plt.title('Integral Curves of dy/dx = x')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid(True)
plt.show()