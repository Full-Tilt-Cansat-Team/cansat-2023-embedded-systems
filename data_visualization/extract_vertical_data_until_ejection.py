#Extracts time and vertical data from until index of ejection
#This script will then interpolate the data to 10x the original data points, and save the data to a CSV file

import matplotlib.pyplot as plt
import csv

time_index = 0
altitude_index = 1

csv_file = open('C:\\Users\\techn\Documents\\GitHub\\cansat-2023\\data_visualization\\flight_simulation_data_launch_only.csv', 'r')

data = list(csv.reader(csv_file, delimiter=','))

time_data = []
altitude_data = []

#record only until ejection
ejection_index = 270

for row_index in range(ejection_index):
    try:
        time_data.append(float(data[row_index][time_index]))
        altitude_data.append(float(data[row_index][altitude_index]))
    except:
        print("Comment Frame")

csv_file.close()

#Interpolate the data to 10x the original data points
time_data_interpolated = []
altitude_data_interpolated = []

for i in range(len(time_data) - 1):
    time_data_interpolated.append(time_data[i])
    altitude_data_interpolated.append(altitude_data[i])
    time_data_interpolated.append((time_data[i] + time_data[i + 1]) / 2)
    altitude_data_interpolated.append((altitude_data[i] + altitude_data[i + 1]) / 2)

#Save the data to a CSV file

csv_file = open('C:\\Users\\techn\Documents\\GitHub\\cansat-2023\\data_visualization\\flight_simulation_data_launch_only_vertical.csv', 'w')

for i in range(len(time_data)):
    csv_file.write(str(time_data_interpolated[i]) + ',' + str(altitude_data_interpolated[i]) +"\n")

csv_file.close()

#Graph the interpolated data out in 2D, just to make sure it looks right, using matplotlib
fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(time_data_interpolated, altitude_data_interpolated, label='Flight Path')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Altitude (m)')

plt.show()