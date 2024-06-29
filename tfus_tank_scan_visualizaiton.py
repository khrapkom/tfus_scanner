import serial
import time
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from matplotlib.widgets import Slider
import threading

data = pd.read_csv('sensor_data.csv')

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

#scatter = ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)

mappable = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
mappable.set_array(sensor_values)
cbar = plt.colorbar(mappable, ax=ax, label='Sensor Value')

def update(val):
    z_index = int(slider.val)
    ax.clear()
    surf = ax.plot_surface(xi[:, :, z_index], yi[:, :, z_index], zi[:, :, z_index], facecolors=plt.cm.viridis(norm(sensor_values_grid[:, :, z_index])), shade=False)
    #scatter = ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)
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
#ax.scatter(x, y, z, c=sensor_values, cmap='viridis', norm=norm)

plt.show()
