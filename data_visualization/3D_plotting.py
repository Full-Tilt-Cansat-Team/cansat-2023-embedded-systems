#This script takes in a CSV file and plots the data in 3D using matplotlib.

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

with open('C:\\Users\\techn\Documents\\GitHub\\cansat-2023\\data_visualization\\flight_simulation_data_launch_only.csv', 'r') as csvfile:
    data = list(csv.reader(csvfile, delimiter=','))

x_data = []
y_data = []
z_data = []

for row in data:
    try:
        x_data.append(float(row[6]))
        y_data.append(float(row[7]))
        z_data.append(float(row[1]))
    except:
        print("Comment Frame")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_data, y_data, z_data, c='r', marker='o')

ax.set_xlabel('Meters East of Launch Site')
ax.set_ylabel('Meter North of Launch Site')
ax.set_zlabel('Altitude (m)')

plt.show()