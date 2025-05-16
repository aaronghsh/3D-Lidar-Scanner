import serial
import open3d as o3d
import numpy as np
import time
import math

#  intial parameters
num_slices = 3                   # (CONFIG) Total number of vertical layers to scan (slices)
num_segments = 32                # (CONFIG) Number of distance measurements (points) per layer
z_step_height = 100              # (CONFIG) Vertical height between each slice/layer (in mm or chosen units)
current_height = 0               # (CONFIG) Tracks the current Z height during scanning

# serial communication setup
ser = serial.Serial('COM4', 115200, timeout=5)  # (CONFIG) O
print("starting:", ser.name)

# Clear serial buffers before communication
ser.reset_input_buffer()
ser.reset_output_buffer()

# Container to hold all 3D coordinate points for the full scan
all_scan_points = []

print("waiting for PJ0 button press")

# wait for user for scan start (button press)
while True:
    incoming_msg = ser.readline().decode().strip()

    if incoming_msg == "A":
        print("button detected, starting scan ")
        break

# start scanning each layer 
for slice_index in range(num_slices):

    current_layer_points = []  # Stores points for this individual slice
    collected_points = 0       # Counter to track how many points received for this slice
    angle_tick = 0             # Index representing angle step
    distance_value = 0         # Measured radial distance from sensor

    # Collect one full circular scan (num_segments readings)
    while collected_points < num_segments:
        line = ser.readline().decode().strip()

        if line != 'A':
            print("Measurment:", line)

        parts = line.split(',')

        # Expecting each line to contain: angle index, measured distance
        if len(parts) == 2:
            angle_tick = int(parts[0])               # Index in circular rotation
            distance_value = float(parts[1])         # Distance reading from VL53L1X sensor

            # Convert to radians. Each step assumed to be 11.25° apart (360 / 32)
            angle_rad = angle_tick * 11.25 * (math.pi / 180.0)

            # Flip rotation direction on every other layer to ensure alignment (optional)
            if slice_index % 2 != 0:
                angle_rad = (2 * math.pi) - angle_rad

            # Convert polar to Cartesian coordinates (x, y, z)
            x = distance_value * math.cos(angle_rad)
            y = distance_value * math.sin(angle_rad)
            z = slice_index * z_step_height

            current_layer_points.append((x, y, z))
            collected_points += 1

    # Wait for SCAN_DONE signal from MCU before moving to next layer
    while True:
        done_message = ser.readline().decode().strip()

        if done_message == "scan_done":
            print(f"Slice {slice_index} scan finished")
            break

    # Append all points from current slice to full scan list
    all_scan_points.extend(current_layer_points)

# === End Serial Communication ===
ser.close()

# =save scan data to .xyz file=
with open("3Dvisual.xyz", "w") as f:
    for x, y, z in all_scan_points:
        f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

# visualize the Scan Using Open3D 
# convert list of tuples to NumPy array for Open3D
point_array = np.array(all_scan_points)

# create PointCloud object and assign points
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_array)

# create LineSet for connectivity between points 
line_connections = []

# vonnect points within each slice to form circular ring
for slice_index in range(num_slices):
    offset = slice_index * num_segments

    for i in range(num_segments - 1):
        line_connections.append([offset + i, offset + i + 1])

    line_connections.append([offset + num_segments - 1, offset])  # close the loop

#  onnect corresponding points between slices (vertical lines)
for point_index in range(num_segments):
    for slice_index in range(num_slices - 1):  # skip last slice
        current_point = slice_index * num_segments + point_index
        next_slice_point = (slice_index + 1) * num_segments

        #  connect top-to-bottom vertically with mirrored wraparound
        if point_index == 0:
            line_connections.append([current_point, next_slice_point + point_index])
        else:
            line_connections.append([current_point, next_slice_point + num_segments - point_index])

# initialize and assign line geometry for visualization
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(point_array)
line_set.lines = o3d.utility.Vector2iVector(line_connections)
line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in line_connections])  # red lines

# display the final point cloud and connected lines in a 3D viewer
o3d.visualization.draw_geometries([pcd, line_set])
