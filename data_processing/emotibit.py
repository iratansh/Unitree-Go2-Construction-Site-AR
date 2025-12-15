"""Quick-look plotting utility for EmotiBit CSV exports.

Set ``file_path`` to the raw CSV produced by the EmotiBit utility. The
script normalizes line lengths, plots each numeric sensor channel, and
prints a summary of row counts per sensor type so that recording quality
can be assessed before ingestion into a larger pipeline.
"""

import matplotlib.pyplot as plt
from collections import Counter
from pathlib import Path

file_path = ""  # Path to the EmotiBit CSV export

if not file_path:
    raise ValueError("Set 'file_path' to the EmotiBit CSV you want to inspect")

csv_path = Path(file_path)
if not csv_path.exists():
    raise FileNotFoundError(f"EmotiBit export not found: {csv_path}")

# Read raw lines from the CSV (handles inconsistent rows)
with csv_path.open("r", encoding="utf-8") as file:
    lines = file.readlines()

# Split each line by comma
split_lines = [line.strip().split(",") for line in lines]

sensor_types = list({row[3] for row in split_lines if len(row) > 3})
print("Detected Sensor Types:", sensor_types)
numerical_sensors = ["EA", "EL", "HR", "T1", "AX", "AY", "AZ", "GX", "GY", "GZ"]

sensor_labels = {
    "EA": ("Electrodermal Activity (EDA)", "EDA (uS)"),
    "EL": ("Smoothed Electrodermal Activity", "EDA (uS)"),
    "HR": ("Heart Rate", "Heart Rate (BPM)"),
    "T1": ("Skin Temperature", "Temperature (deg C)"),
    "AX": ("Accelerometer X-Axis", "Acceleration (g)"),
    "AY": ("Accelerometer Y-Axis", "Acceleration (g)"),
    "AZ": ("Accelerometer Z-Axis", "Acceleration (g)"),
    "GX": ("Gyroscope X-Axis", "Angular Velocity (deg/s)"),
    "GY": ("Gyroscope Y-Axis", "Angular Velocity (deg/s)"),
    "GZ": ("Gyroscope Z-Axis", "Angular Velocity (deg/s)"),
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
