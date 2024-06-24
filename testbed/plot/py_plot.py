import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

file_path: str = ""
data_frame: pd.DataFrame


def set_file_path(path: str) -> None:
    global file_path
    file_path = path


def create_dataframe() -> pd.DataFrame:
    global data_frame
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
    data_frame = df


def plot_trace():
    trace_points = data_frame[["drone_id", "position_x", "position_y"]]
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot the movement traces
    for drone_id in trace_points["drone_id"].unique():
        drone_data = trace_points[trace_points["drone_id"] == drone_id]
        ax.plot(
            drone_data["position_x"],
            drone_data["position_y"],
            marker="",
            label=f"Drone {drone_id}",
        )
    plt.show()


def plot_heatmap(bin_size: str):
    bin_size = int(bin_size)
    # Define the grid for the heatmap
    x_bins = np.linspace(
        data_frame["position_x"].min(), data_frame["position_x"].max(), bin_size
    )
    y_bins = np.linspace(
        data_frame["position_y"].min(), data_frame["position_y"].max(), bin_size
    )

    # Compute the 2D histogram
    heatmap, xedges, yedges = np.histogram2d(
        data_frame["position_x"], data_frame["position_y"], bins=[x_bins, y_bins]
    )

    fig, ax = plt.subplots(figsize=(12, 6))
    # Flip the heatmap array to match the orientation
    heatmap = np.flipud(heatmap.T)

    # Plot the heatmap
    im = ax.imshow(
        heatmap,
        aspect="auto",
        cmap="plasma",
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    )
    fig.colorbar(im, ax=ax)
    plt.show()
