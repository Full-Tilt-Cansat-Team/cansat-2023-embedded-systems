#This script extracts the time and vertical data from the CSV file generate by extract_vertical_data_until_ejection.py
#it then simulates the vertical data after the ejction, based on the gravitational acceleration of eath and the terminal velocity of the various flight phases
#It will then save the old and new data combined to a CSV file, and plot the data in 2D using matplotlib

import matplotlib.pyplot as plt
import csv

time_index = 0
altitude_index = 1

csv_file = open('C:\\Users\\techn\Documents\\GitHub\\cansat-2023\\data_visualization\\flight_simulation_data_launch_only_vertical.csv', 'r')

data = list(csv.reader(csv_file, delimiter=','))

time_data = []
altitude_data = []

for row in data:
    try:
        time_data.append(float(row[time_index]))
        altitude_data.append(float(row[altitude_index]))
    except:
        print("Comment Frame")

csv_file.close()

earth_acceleration = 9.81

#Simulate the vertical data after the ejction, based on the gravitational acceleration of eath and the terminal velocity of the various flight phases
#The first stage of the flight occurs from release until the probe is dropped (altitude < 500m). This stage is assumed to have a terminal velocity of 15m/s
#The second stage of the flight occurs from the probe being dropped until the parachute is deployed (altitude < 200m). This stage is assumed to have a terminal velocity of 20m/s
#The third stage of the flight occurs from the parachute being deployed until the probe lands (altitude < 0m). This stage is assumed to have a terminal velocity of 5m/s
#Data is simulated at 10Hz

#First stage
stage_ending_altitude = 500
stage_terminal_velocity = 15
current_altitude = altitude_data[-1]
current_net_acceleration = earth_acceleration
current_vertical_velocity = 0

for i in range(1000):
    current_vertical_velocity = current_vertical_velocity + current_net_acceleration / 10
    if current_vertical_velocity > stage_terminal_velocity:
        current_vertical_velocity = stage_terminal_velocity
    current_altitude = current_altitude - current_vertical_velocity / 10
    if current_altitude < stage_ending_altitude:
        break
    time_data.append(time_data[-1] + 0.1)
    altitude_data.append(current_altitude)

#Second stage
stage_ending_altitude = 200
stage_terminal_velocity = 20
current_altitude = altitude_data[-1]
current_net_acceleration = earth_acceleration

for i in range(1000):
    current_vertical_velocity = current_vertical_velocity + current_net_acceleration / 10
    if current_vertical_velocity > stage_terminal_velocity:
        current_vertical_velocity = stage_terminal_velocity
    current_altitude = current_altitude - current_vertical_velocity / 10
    if current_altitude < stage_ending_altitude:
        break
    time_data.append(time_data[-1] + 0.1)
    altitude_data.append(current_altitude)

#Third stage
stage_ending_altitude = 0
stage_terminal_velocity = 5
current_altitude = altitude_data[-1]
current_net_acceleration = earth_acceleration

for i in range(1000):
    current_vertical_velocity = current_vertical_velocity + current_net_acceleration / 10
    if current_vertical_velocity > stage_terminal_velocity:
        current_vertical_velocity = stage_terminal_velocity
    current_altitude = current_altitude - current_vertical_velocity / 10
    if current_altitude < stage_ending_altitude:
        break
    time_data.append(time_data[-1] + 0.1)
    altitude_data.append(current_altitude)

#Plot the data in 2D
plt.plot(time_data, altitude_data, label='Flight Path')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Flight Path')
plt.legend()
plt.show()

#Save the data to a CSV file
with open('C:\\Users\\techn\Documents\\GitHub\\cansat-2023\\data_visualization\\flight_simulation_data_launch_only_vertical_with_post_ejection_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Altitude (m)"])
    for i in range(len(time_data)):
        writer.writerow([time_data[i], altitude_data[i]])
