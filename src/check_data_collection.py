import numpy as np
import cv2
import os
import random

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the paths to the saved data
data_collection_path = '/home/nanz/catkin_ws/src/rl_data/collected_data/'
data_file_path = os.path.join(data_collection_path, 'episode18.npy')

# Load the saved data
data = np.load(data_file_path, allow_pickle=True)

# Select 5 random observations
#random_indices = random.sample(range(len(data)), min(100, len(data)))


# Extract joint positions (x, y, z) for each observation
print(len(data))
joint_positions = []
for observation in data:
    joint_positions.append(observation['joint_positions'])
joint_positions = np.array(joint_positions)

# Plot joint positions in 3D as a line plot
fig = plt.figure()


# Display concatenated image with specified FPS
fps = 30

# Calculate delay time based on FPS
delay = int(1000 / fps)

# Define the scaling factor for resizing the displayed image
scaling_factor = 5

# Display main and wrist images for each selected observation
# Display main and wrist images for each selected observation
for idx in range(len(data)):
    observation = data[idx]
    main_image_data = observation['main_image']
    wrist_image_data = observation['wrist_image']

    main_image = np.reshape(main_image_data, (48, 48, 3))
    wrist_image = np.reshape(wrist_image_data, (48, 48, 3))

    # Concatenate images side by side
    concatenated_image = np.concatenate((main_image, wrist_image), axis=1)

    # Resize the concatenated image
    resized_image = cv2.resize(concatenated_image, None, fx=scaling_factor, fy=scaling_factor)

    # Plot resized image
    plt.imshow(resized_image)
    plt.axis('off')
    plt.pause(0.5)  # Increase pause time to slow down display

plt.show()

