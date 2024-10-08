import re
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance_matrix
import os
import json

matplotlib.use("Agg")

global df
global output_path
global file_name
global parsed_first_line

global border_dimensions
global drone_spawn_x
global drone_spawn_y
global max_time


def parse_log_line(line):
    # Use regex to extract the JSON part from the line
    match = re.search(r"\{.*\}", line)
    if match:
        json_str = match.group(0)
        try:
            # Parse the JSON string into a dictionary
            data = json.loads(json_str)
            return data
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON: {e}")
            return None
    else:
        print("No JSON found in the line")
        return None


def read_first_line(file_path):
    try:
        with open(file_path, "r") as file:
            first_line = file.readline().strip()
            return first_line
    except Exception as e:
        print(f"Error reading file: {e}")
        return None


def get_sim_data(file_path):
    global parsed_first_line
    global border_dimensions
    global drone_spawn_x
    global drone_spawn_y
    global max_time
    first_line = read_first_line(file_path)
    if first_line:
        parsed_first_line = parse_log_line(first_line)
        if parsed_first_line:
            border_dimensions = parsed_first_line.get("border_dimensions", None)
            drone_spawn = parsed_first_line.get("drone_spawn_position", None)
            drone_spawn_x = drone_spawn[0]
            drone_spawn_y = drone_spawn[1]
            max_time = parsed_first_line.get("time_limit", None)
    else:
        return None


def set_file_name(name: str):
    global file_name
    file_name = name[:-10]


def set_output_path(path: str):
    global output_path
    output_path = path


def create_dataframe(file_path: str) -> pd.DataFrame:
    global df

    print(os.path.abspath(__file__))
    with open(file_path, "r") as file:
        data = file.read()

    pattern = re.compile(
        r'\[(\d+\.\d+)\] \[salsa::Drone (\d+)\] {"position":\[(.*?),(.*?)\],"velocity":\[(.*?),(.*?)\]}'
        r'|\[(\d+\.\d+)\] \[Sim 0\] {"targets_found":(\d+)}'
    )

    matches = pattern.findall(data)
    records = []
    for match in matches:
        if match[0]:  # This is a drone entry
            record = {
                "timestamp": float(match[0]),
                "drone_id": int(match[1]),
                "position_x": float(match[2]),
                "position_y": float(match[3]),
                "velocity_x": float(match[4]),
                "velocity_y": float(match[5]),
                "targets_found": None,  # No targets data in drone entries
            }
        else:  # This is a simulation targets found entry
            record = {
                "timestamp": float(match[6]),
                "drone_id": None,
                "position_x": None,
                "position_y": None,
                "velocity_x": None,
                "velocity_y": None,
                "targets_found": int(match[7]),
            }
        records.append(record)
    # Creating DataFrame from records
    data_frame = pd.DataFrame(records)
    df = data_frame
    return data_frame


def get_axis_steps(border_size: str) -> tuple:
    first_number = float(border_size[0])
    border_size = float(border_size)

    if border_size >= 1000:
        step_round_base = 1000
    elif border_size >= 100:
        step_round_base = 100
    else:
        step_round_base = 10

    step_size = round(border_size / first_number / step_round_base) * step_round_base

    if step_size >= 1000:
        minor_round_base = 100
    elif step_size >= 100:
        minor_round_base = 10
    else:
        minor_round_base = 1

    minor_size = round(step_size / first_number / minor_round_base) * minor_round_base

    return step_size, minor_size


def plot_targets_found(data_frame, ax):
    sim_data = data_frame[data_frame["targets_found"].notna()]

    # Ensure data is sorted by time
    sim_data = sim_data.sort_values("timestamp")

    # Convert timestamp to seconds from the start of the log
    sim_data["time_seconds"] = sim_data["timestamp"] - sim_data["timestamp"].iloc[0]
    sim_data["targets_percentage"] = (sim_data["targets_found"] / 500) * 100

    # Plotting
    ax.set_xlim(0, max_time)
    ax.set_ylim(0, 100)
    ax.plot(
        sim_data["time_seconds"],
        sim_data["targets_percentage"],
        marker="",
        color="g",
    )
    ax.grid(True)
    ax.set_ylabel("Targets Found (%)")
    ax.set_xlabel("Time (s)")

    return ax

    plt.show()


def plot_targets_found_wrapper():
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
    ax = plot_targets_found(df, ax)
    print(f"{file_name}")
    fig.savefig(f"{output_path}/{file_name}targets_found.pdf")
    plt.close(fig)


def plot_speed(data_frame, ax):
    speed_points = data_frame[["drone_id", "velocity_x", "velocity_y", "timestamp"]]
    # Plot the movement traces
    speed_points["speed"] = np.sqrt(
        data_frame["velocity_x"] ** 2 + speed_points["velocity_y"] ** 2
    )

    speed_points["timestamp"] = pd.to_datetime(
        data_frame["timestamp"], unit="s", origin="unix"
    )
    speed_points.set_index("timestamp", inplace=True)

    speed_stats = (
        speed_points["speed"].resample("1S").agg(["min", "mean", "max", "std"]).dropna()
    )

    # Reset index if you want 'timestamp' as a column for plotting
    speed_stats.reset_index(inplace=True)

    ax.set_xlim(0, max_time)
    ax.set_ylim(0, speed_stats["mean"].max() + speed_stats["std"].max())
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlabel("Time (s)")

    ax.plot(speed_stats.index, speed_stats["mean"], label="avg. speed", color="blue")

    ax.fill_between(
        speed_stats.index,
        speed_stats["mean"] - speed_stats["std"],
        speed_stats["mean"] + speed_stats["std"],
        color="blue",
        label="speed std. dev",
        alpha=0.2,
    )
    return ax
    plt.grid(True)
    plt.savefig("drone_speed.pdf")
    plt.show()


def plot_speed_wrapper():
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
    ax = plot_speed(df, ax)
    print(f"{file_name}")
    fig.savefig(f"{output_path}/{file_name}/drone_speed.pdf")
    plt.close(fig)


def plot_speed_per_drone(data_frame):
    speed_points = data_frame[["drone_id", "velocity_x", "velocity_y", "timestamp"]]

    # Calculate the speed
    speed_points["speed"] = np.sqrt(
        speed_points["velocity_x"] ** 2 + speed_points["velocity_y"] ** 2
    )

    # Ensure timestamp is set as the index and is a datetime type if it's not already
    speed_points["timestamp"] = pd.to_datetime(
        speed_points["timestamp"], unit="s", origin="unix"
    )
    speed_points.set_index("timestamp", inplace=True)

    # Create a figure and axes
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")

    # Get unique drone IDs
    drone_ids = speed_points["drone_id"].unique()

    for drone_id in drone_ids:
        # Filter data for the current drone
        drone_data = speed_points[speed_points["drone_id"] == drone_id]

        # Resample and calculate min, mean, and max speed for each 100 milliseconds for this drone
        speed_stats = (
            drone_data["speed"].resample("100L").agg(["min", "mean", "max"]).dropna()
        )

        # Plotting the results for each drone
        ax.plot(
            speed_stats.index,
            speed_stats["mean"],
            label=f"Average Speed Drone {drone_id}",
        )
        ax.fill_between(
            speed_stats.index,
            speed_stats["min"],
            speed_stats["max"],
            alpha=0.3,
            label=f"Min-Max Range Drone {drone_id}",
        )

    # Configure plot
    ax.set_xlabel("Timestamp")
    ax.set_ylabel("Speed (units per 100 milliseconds)")
    ax.set_title("Speed Statistics Over Time per Drone (Averaged per 100 Milliseconds)")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_nearest_neighbor_distance(data_frame, ax):
    filtered_data = data_frame[data_frame["drone_id"].notna()]

    # Convert timestamps to datetime objects and set as index
    filtered_data["timestamp"] = pd.to_datetime(
        filtered_data["timestamp"], unit="s", origin="unix"
    )
    filtered_data.set_index("timestamp", inplace=True)
    # Prepare to collect stats for each timestamp
    stats = []
    for timestamp, group in filtered_data.groupby(level=0):
        if len(group) > 1:  # Need at least two drones to calculate distances
            # Calculate the pairwise distance matrix and set diagonal to infinity
            distances = distance_matrix(
                group[["position_x", "position_y"]], group[["position_x", "position_y"]]
            )
            np.fill_diagonal(distances, np.inf)

            # Calculate the minimum distances for nearest neighbor
            min_distances = np.min(distances, axis=1)
            stats.append(
                {
                    "timestamp": timestamp,
                    "min": np.min(min_distances),
                    "mean": np.mean(min_distances),
                    "max": np.max(min_distances),
                    "std": np.std(min_distances),
                }
            )
            if np.std(min_distances) is None:
                print("Standard deviation is None")

    if stats:
        stats_df = pd.DataFrame(stats)

        # Resample the statistics to improve smoothness in the plot
        resampled_stats = (
            stats_df.resample("1S", on="timestamp")
            .agg({"min": "min", "mean": "mean", "max": "max", "std": "mean"})
            .dropna()
        )
        resampled_stats.reset_index(inplace=True)
        # Plot the average distances and the standard deviation
        ax.plot(
            resampled_stats.index,
            resampled_stats["mean"],
            label="Average Distance",
            color="red",
        )
        ax.fill_between(
            resampled_stats.index,
            resampled_stats["mean"] - resampled_stats["std"],
            resampled_stats["mean"] + resampled_stats["std"],
            color="red",
            alpha=0.2,
            label="Std. Dev of Distance",
        )

        ax.set_xlim(
            resampled_stats.index[0],
            max_time,
        )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Nearest Neighbor Distance (m)")
    else:
        print("Not enough data points for meaningful distance calculations.")

    return ax
    plt.show()


def plot_drone_distances_wrapper():
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
    ax = plot_nearest_neighbor_distance(df, ax)
    print(f"{file_name}")
    fig.savefig(f"{output_path}/{file_name}/nearest_neighbor_distance.pdf")
    plt.close(fig)


def plot_trace(data_frame, ax, border_size: str):
    trace_points = data_frame[["drone_id", "position_x", "position_y"]]
    step_size, minor_size = get_axis_steps(border_size)
    border_size = float(border_size)
    ax.set_xlim(0, border_size)
    ax.set_ylim(0, border_size)
    ax.set_yticks(np.arange(0, border_size + step_size, step_size))
    ax.set_xticks(np.arange(0, border_size + step_size, step_size))
    ax.set_yticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    ax.set_xticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    ax.set_aspect("equal", adjustable="box")
    for drone_id in trace_points["drone_id"].unique():
        drone_data = trace_points[trace_points["drone_id"] == drone_id]
        ax.plot(
            drone_data["position_x"],
            drone_data["position_y"],
            marker="",
            label=f"Drone {drone_id}",
            linewidth=1,
        )
    ax.plot(
        drone_spawn_x,
        drone_spawn_y,
        marker="x",
        color="black",
        markersize=10,
        mew=4,
        linewidth=4,
        label="Spawn",
    )
    ax.set_ylabel("Height (m)")
    ax.set_xlabel("Width (m)")
    return ax


def plot_trace_wrapper():
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
    ax = plot_trace(df, ax, str(border_dimensions[0]))
    print(f"{file_name}")
    fig.savefig(f"{output_path}/{file_name}/drone_trace.pdf")
    plt.close(fig)


def plot_heatmap(data_frame, ax, bin_size: str, border_size: str):
    bin_size = int(bin_size)
    min_boundary = 0
    step_size, minor_size = get_axis_steps(border_size)
    border_size = float(border_size)
    ax.set_xlim(0, border_size)
    ax.set_ylim(0, border_size)
    ax.set_yticks(np.arange(0, border_size + step_size, step_size))
    ax.set_xticks(np.arange(0, border_size + step_size, step_size))
    ax.set_yticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    ax.set_xticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    # Define the grid for the heatmap
    x_bins = np.linspace(min_boundary, border_size, bin_size)
    y_bins = np.linspace(min_boundary, border_size, bin_size)

    heatmap, xedges, yedges = np.histogram2d(
        data_frame["position_x"], data_frame["position_y"], bins=[x_bins, y_bins]
    )
    heatmap = np.clip(heatmap, 0, 100)

    heatmap = np.flipud(heatmap.T)

    im = ax.imshow(
        heatmap,
        aspect="equal",
        cmap="plasma",
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    )
    # fig.colorbar(im, ax=ax, pad=0.001, location="bottom")
    ax.grid(False)
    ax.grid(False, which="minor")
    ax.set_ylabel("Height (m)")
    ax.set_xlabel("Width (m)")

    return im, ax


def plot_heatmap_wrapper(bin_size: str):
    fig, ax = plt.subplots(figsize=(10, 6), layout="constrained")
    im, ax = plot_heatmap(df, ax, bin_size, str(border_dimensions[0]))
    fig.savefig(f"{output_path}/{file_name}/drone_heatmap.pdf")
    plt.close(fig)
