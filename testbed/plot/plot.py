import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

file_path = "new_log.log"

with open(file_path, "r") as file:
    data = file.read()

pattern = re.compile(
    r'\[(.*?)\] \[swarm::Drone (\d+)\] {"position":\[(.*?),(.*?)\],"velocity":\[(.*?),(.*?)\]}'
)

matches = pattern.findall(data)
df = pd.DataFrame(
    matches,
    columns=[
        "timestamp",
        "drone_id",
        "position_x",
        "position_y",
        "velocity_x",
        "velocity_y",
    ],
)
df["position_x"] = df["position_x"].astype(float)
df["position_y"] = df["position_y"].astype(float)
df["velocity_x"] = df["velocity_x"].astype(float)
df["velocity_y"] = df["velocity_y"].astype(float)
df["drone_id"] = df["drone_id"].astype(int)
df["timestamp"] = pd.to_datetime(df["timestamp"], format="%m-%d %H:%M:%S.%f")

trace_points = df[["drone_id", "position_x", "position_y"]]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 5))

drone_pos = pd.DataFrame()

# Plot the movement traces
for drone_id in trace_points["drone_id"].unique():
    drone_data = trace_points[trace_points["drone_id"] == drone_id]
    ax1.plot(
        drone_data["position_x"],
        drone_data["position_y"],
        marker="",
        label=f"Drone {drone_id}",
    )
bin_size = 100

# Define the grid for the heatmap
x_bins = np.linspace(df["position_x"].min(), df["position_x"].max(), bin_size)
y_bins = np.linspace(df["position_y"].min(), df["position_y"].max(), bin_size)

# Compute the 2D histogram
heatmap, xedges, yedges = np.histogram2d(
    df["position_x"], df["position_y"], bins=[x_bins, y_bins]
)

# Flip the heatmap array to match the orientation
heatmap = np.flipud(heatmap.T)

# Plot the heatmap
im = ax2.imshow(
    heatmap,
    aspect="auto",
    cmap="plasma",
    extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
)
fig.colorbar(im, ax=ax2)

# Show the plots
plt.tight_layout()
plt.show()

fig.savefig("plot.pdf", bbox_inches="tight")

# Data from the user: differences in seconds between the pairs of times
time_differences = []

# Convert the time differences into the format required (divided by 100)
y_values = [x / 100.0 for x in time_differences]

# X-axis values: 2^i for i from 1 to 10
x_labels = [2**i for i in range(1, 11)]
print(x_labels)
# Set up the plot
fig = plt.figure(figsize=(10, 6))
plt.plot(x_labels, y_values, marker="o")

# Setting the x-axis to be logarithmic scale
plt.xscale("log")
plt.xticks(x_labels, x_labels)
# Custom ticks for the y-axis
plt.yscale("log")
plt.yticks(
    [10**-2, 10**-1, 10**0, 10**1, 10**2],
    [r"$10^{-2}$", r"$10^{-1}$", r"$10^{0}$", r"$10^{1}$", r"$10^{2}$"],
)

# Show the plot
plt.grid(True, which="both", ls="--")
plt.show()
fig.savefig("plot2.pdf", bbox_inches="tight")
