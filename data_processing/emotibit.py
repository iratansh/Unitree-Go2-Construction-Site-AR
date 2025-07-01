import pandas as pd
import matplotlib.pyplot as plt
from collections import Counter

# ---- Step 1: Load and parse the CSV ----
file_path = ""  

# Read raw lines from the CSV (handles inconsistent rows)
with open(file_path, "r") as file:
    lines = file.readlines()

# Split each line by comma
split_lines = [line.strip().split(",") for line in lines]

sensor_types = list({row[3] for row in split_lines if len(row) > 3})
print("Detected Sensor Types:", sensor_types)
numerical_sensors = ["EA", "EL", "HR", "T1", "AX", "AY", "AZ", "GX", "GY", "GZ"]

sensor_labels = {
    "EA": ("Electrodermal Activity (EDA)", "EDA (µS)"),
    "EL": ("Smoothed Electrodermal Activity", "EDA (µS)"),
    "HR": ("Heart Rate", "Heart Rate (BPM)"),
    "T1": ("Skin Temperature", "Temperature (°C)"),
    "AX": ("Accelerometer X-Axis", "Acceleration (g)"),
    "AY": ("Accelerometer Y-Axis", "Acceleration (g)"),
    "AZ": ("Accelerometer Z-Axis", "Acceleration (g)"),
    "GX": ("Gyroscope X-Axis", "Angular Velocity (°/s)"),
    "GY": ("Gyroscope Y-Axis", "Angular Velocity (°/s)"),
    "GZ": ("Gyroscope Z-Axis", "Angular Velocity (°/s)"),
}

sensor_data = {sensor: [] for sensor in numerical_sensors}
time_data = {sensor: [] for sensor in numerical_sensors}

for row in split_lines:
    if len(row) >= 7 and row[3] in numerical_sensors:
        try:
            timestamp = int(row[0]) / 1000.0  # Convert ms to seconds
            value = float(row[6])
            sensor_data[row[3]].append(value)
            time_data[row[3]].append(timestamp)
        except ValueError:
            continue

for sensor in numerical_sensors:
    if sensor_data[sensor]:
        title, ylabel = sensor_labels.get(sensor, (sensor, "Value"))
        plt.figure(figsize=(10, 3))
        plt.plot(time_data[sensor], sensor_data[sensor], label=sensor)
        plt.title(f"{title} Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

sensor_counts = Counter(row[3] for row in split_lines if len(row) > 3)
print("\nSensor Row Counts:")
for sensor, count in sensor_counts.items():
    print(f"{sensor}: {count}")
