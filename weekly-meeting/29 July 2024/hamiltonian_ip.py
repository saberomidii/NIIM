from numba import cuda
import numpy as np
import math

@cuda.jit
def calculate_pi_gpu(random_x, random_y, count_inside_circle):
    idx = cuda.grid(1)
    if idx < random_x.size:
        x = random_x[idx]
        y = random_y[idx]
        if math.sqrt(x ** 2 + y ** 2) <= 1.0:
            count_inside_circle[idx] = 1

def main():
    num_samples =900000000

    # Generate random points
    random_x = np.random.rand(num_samples).astype(np.float32)
    random_y = np.random.rand(num_samples).astype(np.float32)
    count_inside_circle = np.zeros(num_samples, dtype=np.int32)

    # Transfer data to the GPU
    random_x_device = cuda.to_device(random_x)
    random_y_device = cuda.to_device(random_y)
    count_inside_circle_device = cuda.to_device(count_inside_circle)

    # Define the number of threads in a block and the number of blocks in a grid
    threads_per_block = 256
    blocks_per_grid = (num_samples + (threads_per_block - 1)) // threads_per_block

    # Launch the kernel
    calculate_pi_gpu[blocks_per_grid, threads_per_block](random_x_device, random_y_device, count_inside_circle_device)

    # Copy the result back to the host
    count_inside_circle_device.copy_to_host(count_inside_circle)

    # Calculate the value of π
    pi_estimate = (4.0 * np.sum(count_inside_circle)) / num_samples
    print(f"Estimated value of π: {pi_estimate}")

if __name__ == "__main__":
    main()