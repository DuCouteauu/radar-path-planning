import numpy as np
import matplotlib


# Radar System Settings
arduino_port = 'COM6'
arduino_servo_pin = 9
arduino_servo_power_pin = 8
rotation_increment = 15
front_radar_port = 'COM5'
back_radar_port = 'COM4'
min_distance = 0
radar_field_size = 70  # In degrees
radar_increment = 20
images_per_rotation = 1
radar_range_interval = [1, 6]  # In meters
servo_offset = 7
rotations_per_scan = 19

sim_rotation_delay = 0.2
connected_rotation_delay = 0.5

downsampling_factor = 4
running_average_factor = 0  # Adjust accordingly
refresh_rate = 5

# Arduino Motor Control Pins
pin_front_right = [5, 4]
pin_front_left = [7, 6]
pin_back_right = [11, 10]
pin_back_left = [13, 12]
speed_pin = 3

# Peak Detection Settings
peak_distance = 400  # min distance between peaks, in samples
peak_width = 5  # min width of a peak
height = 220  # min height of a peak
threshold = 0.5
max_peak_change_per_frame = 1

hit_maximum = 10  # Number of hits required to reach max color value on grid


# Grid Settings
max_range = radar_range_interval[1]
grid_range = 2*max_range
grid_size = 17  # nxn, must be odd
grid_size_internal = grid_size - 2  # No longer needed
grid_center = (int((grid_size - 1)/2), int((grid_size - 1)/2))
cell_size = 2 * max_range / grid_size
# Generate lists of cell ranges
cell_ranges = np.linspace(-max_range, max_range, grid_size + 1)

# Grid Color Settings
colors = ["white", "salmon", "red", "tomato", "firebrick", "purple"]
nodes = [0.0, 0.2, 0.4, 0.8, 0.9, 1.0]
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("cmap", list(zip(nodes, colors)))
cmap_final = matplotlib.colors.LinearSegmentedColormap.from_list("cmap_final", list(zip([0.0, 0.7, 1.0], ["blue", "red", "purple"])))


