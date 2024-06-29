import serial
import time
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from matplotlib.widgets import Slider
import math

# Initialize serial connections
ser = serial.Serial('/dev/tty.usbmodem11301', 115200, timeout=1)
ser2 = serial.Serial('/dev/tty.usbmodem11201', 115200, timeout=1)
feed_rate = 100

ser.flushInput()  # Clear any existing input buffer

# Wake up GRBL (if it's in sleep mode)
ser.write(b"\r\n\r\n")
time.sleep(2)  # Wait for GRBL to initialize
ser.flushInput()  # Clear any messages GRBL sent during startup

# Helper function to floor to a specific number of decimals
def floor_to_decimals(number, decimals):
    factor = 10 ** decimals
    return math.floor(number * factor) / factor

# Extract MPos from data string
def extract_mpos(data_string):
    start_index = data_string.find('MPos:') + 5
    end_index = data_string.find(',WPos', start_index)
    position_data = data_string[start_index:end_index]
    next = str(position_data.split(',')).replace('\'', '')
    stripped_string = next.strip('[]')
    parts = stripped_string.split(',')
    try:
        floats = [float(part) for part in parts]
    except:
        return
    return floats

# Move the CNC machine
def move(axis, length, target):
    target = round(target, 3)
    ser.flushInput()
    mapping = ['X', 'Y', 'Z']
    ser.write(b"G91\n")  # Set to relative positioning
    command = f"G1 {mapping[axis]}{length} F{feed_rate}\n"
    ser.write(command.encode())
    ser.write(b"G90\n")  # Set back to absolute positioning

    with open('sensor_data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        while True:  # Query for 1 second
            ser.write(b'?')
            response = ser.readline().decode().strip()
            response = extract_mpos(response)
            sensor_value = ser2.readline().decode().strip()
            if response and len(response) == 3:
                print(response, sensor_value)
                writer.writerow([response[0], response[1], response[2], sensor_value])
                if floor_to_decimals(response[axis], 2) == floor_to_decimals(target, 2) or round(response[axis], 2) == round(target, 2):
                    print("complete")
                    break
            time.sleep(0.001)  # Sleep for 1 millisecond

# Function to estimate total time and track elapsed time
def estimate_time_and_track_progress(total_steps):
    start_time = time.time()
    print(f"Estimated total time: {total_steps * 1 / 60:.2f} minutes")  # Assuming each step takes ~1.5 seconds

    step_counter = 0
    while step_counter < total_steps:
        yield
        step_counter += 1
        if step_counter % 10 == 0:
            elapsed_time = time.time() - start_time
            print(f"Elapsed time after {step_counter} steps: {elapsed_time:.2f} seconds")

# Configuration parameters
plus = True
y_max = 20
z_max = 6
z_step = 0.5
x_max = 15
x_step = 0.125

# Open the output file
with open('sensor_data_DETAILED.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['X', 'Y', 'Z', 'SensorValue'])

# Calculate total steps
total_x_steps = int(x_max / x_step)
total_z_steps = int(z_max / z_step)
total_y_steps = 2 * total_x_steps * total_z_steps  # Each x and z step involves a full y scan
total_steps = total_x_steps * total_y_steps + total_x_steps

# Create a generator for progress tracking
progress_tracker = estimate_time_and_track_progress(total_steps)

# Main loop
current_pos = [0, 0, 0]
while current_pos[0] <= x_max:
    if plus:
        current_pos[1] = current_pos[1] + y_max
        move(1, y_max, current_pos[1])
        plus = False
    else:
        current_pos[1] = current_pos[1] - y_max
        move(1, -y_max, current_pos[1])
        plus = True
    while current_pos[2] < z_max:
        current_pos[2] = current_pos[2] + z_step
        move(2, z_step, current_pos[2])
        if plus:
            current_pos[1] = current_pos[1] + y_max
            move(1, y_max, current_pos[1])
            plus = False
        else:
            current_pos[1] = current_pos[1] - y_max
            move(1, -y_max, current_pos[1])
            plus = True
        next(progress_tracker)  # Update progress every step
    if plus:
        current_pos[1] = current_pos[1] + y_max
        move(1, y_max, current_pos[1])
        plus = False
    else:
        current_pos[1] = current_pos[1] - y_max
        move(1, -y_max, current_pos[1])
        plus = True
    current_pos[0] = current_pos[0] + x_step
    if current_pos[0] > x_max:
        break
    move(0, x_step, current_pos[0])
    current_pos[2] = current_pos[2] - z_max
    move(2, -z_max, current_pos[2])
    next(progress_tracker)  # Update progress every step

# Visualization

data = pd.read_csv('sensor_data_Overnight.csv')

x = data['X']
y = data['Y']
z = data['Z']
sensor_values = data['SensorValue']

xi = np.linspace(x.min(), x.max(), 100)
yi = np.linspace(y.min(), y.max(), 100)
zi = np.linspace(z.min(), z.max(), 100)
xi, yi, zi = np.meshgrid(xi, yi, zi)

sensor_values_grid = griddata((x, y, z), sensor_values, (xi, yi, zi), method='linear')

norm = plt.Normalize(vmin=sensor_values.min(), vmax=sensor_values.max())

initial_z_index = 0

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_surface(xi[:, :, initial_z_index], yi[:, :, initial_z_index], zi[:, :, initial_z_index], facecolors=plt.cm.viridis(norm(sensor_values_grid[:, :, initial_z_index])), shade=False)

scatter = ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)

mappable = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
mappable.set_array(sensor_values)
cbar = plt.colorbar(mappable, ax=ax, label='Sensor Value')

def update(val):
    z_index = int(slider.val)
    ax.clear()
    surf = ax.plot_surface(xi[:, :, z_index], yi[:, :, z_index], zi[:, :, z_index], facecolors=plt.cm.viridis(norm(sensor_values_grid[:, :, z_index])), shade=False)
    scatter = ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f"Z level: {zi[0, 0, z_index]:.2f}")
    plt.draw()

# Slider
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Z level', 0, len(zi[0, 0]) - 1, valinit=initial_z_index, valstep=1)
slider.on_changed(update)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.plot_surface(xi[:, :, initial_z_index], yi[:, :, initial_z_index], zi[:, :, initial_z_index], facecolors=plt.cm.viridis(norm(sensor_values_grid[:, :, initial_z_index])), shade=False)
ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)

plt.show()
