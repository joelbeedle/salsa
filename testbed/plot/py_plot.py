import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance_matrix
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from matplotlib.transforms import ScaledTranslation
import matplotlib.image as mpimg


def create_dataframe(file_path: str) -> pd.DataFrame:
    global data_frame
    with open(file_path, "r") as file:
        data = file.read()

    pattern = re.compile(
        r'\[(\d+\.\d+)\] \[swarm::Drone (\d+)\] {"position":\[(.*?),(.*?)\],"velocity":\[(.*?),(.*?)\]}'
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
    return data_frame


def get_axis_steps(border_size: str) -> tuple:
    first_number = int(border_size[0])
    border_size = int(border_size)

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
    ax.set_xlim(0, 100)
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

    plt.savefig("targets_found.pdf")
    return ax

    plt.show()


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
        speed_points["speed"]
        .resample("100L")
        .agg(["min", "mean", "max", "std"])
        .dropna()
    )

    # Reset index if you want 'timestamp' as a column for plotting
    speed_stats.reset_index(inplace=True)

    ax.set_xlim(0, 100)
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


def plot_speed_per_drone():
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
    ax.set_xlim(0, 100)
    points = data_frame[["drone_id", "position_x", "position_y", "timestamp"]]

    points["timestamp"] = pd.to_datetime(points["timestamp"])
    points.set_index("timestamp", inplace=True)

    stats = []
    for timestamp, grp in points.groupby(level=0):
        if len(grp) > 1:  # Ensure there are at least two drones to compare

            # Calculate a pairwise distance matrix, and then fill the diagonal with
            # infinities, so that we ignore the distance to itself (which would be zero)

            distances = distance_matrix(
                grp[["position_x", "position_y"]], grp[["position_x", "position_y"]]
            )
            np.fill_diagonal(distances, np.inf)

            # Find the distance to the nearest neighbour
            min_distances = np.min(distances, axis=1)

            # Calculate min, mean, and max of these distances
            min_val = np.min(min_distances)
            mean_val = np.mean(min_distances)
            max_val = np.max(min_distances)
            std_val = np.std(min_distances)

            stats.append((timestamp, mean_val, min_val, max_val, std_val))

    stats_df = pd.DataFrame(stats, columns=["timestamp", "mean", "min", "max", "std"])
    stats_df.set_index("timestamp", inplace=True)

    resampled_stats = (
        stats_df.resample("1000L")
        .agg({"min": "min", "mean": "mean", "max": "max", "std": "std"})
        .dropna()
    )

    resampled_stats.reset_index(inplace=True)
    ax.set_ylabel("Distance (m)")
    ax.set_xlabel("Time (s)")
    ax.plot(
        resampled_stats.index,
        resampled_stats["mean"],
        label="avg. distance",
        color="red",
    )
    ax.fill_between(
        resampled_stats.index,
        resampled_stats["mean"] - resampled_stats["std"],
        resampled_stats["mean"] + resampled_stats["std"],
        color="red",
        alpha=0.2,
        label="distance std. dev",
    )
    plt.savefig("nearest_neighbor_distance.pdf")
    return ax
    plt.show()


def plot_trace(data_frame, ax, border_size: str):
    trace_points = data_frame[["drone_id", "position_x", "position_y"]]
    step_size, minor_size = get_axis_steps(border_size)
    border_size = int(border_size)
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
        396.24749755859375,
        299.29345703125,
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


def plot_heatmap(data_frame, ax, bin_size: str, border_size: str):
    bin_size = int(bin_size)
    min_boundary = 0
    max_boundary = 500
    step_size, minor_size = get_axis_steps(border_size)
    border_size = int(border_size)
    ax.set_xlim(0, border_size)
    ax.set_ylim(0, border_size)
    ax.set_yticks(np.arange(0, border_size + step_size, step_size))
    ax.set_xticks(np.arange(0, border_size + step_size, step_size))
    ax.set_yticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    ax.set_xticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    # Define the grid for the heatmap
    x_bins = np.linspace(min_boundary, max_boundary, bin_size)
    y_bins = np.linspace(min_boundary, max_boundary, bin_size)

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


def plot_both(border_size: str, bin_size: str, colorbar_location: str) -> None:
    trace_points = data_frame[["drone_id", "position_x", "position_y"]]
    fig, axs = plt.subplots(1, 2, figsize=(10, 5))
    step_size, minor_size = get_axis_steps(border_size)
    border_size = int(border_size)
    axs[0].set_xlim(0, border_size)
    axs[0].set_ylim(0, border_size)
    axs[0].set_yticks(np.arange(0, border_size + step_size, step_size))
    axs[0].set_xticks(np.arange(0, border_size + step_size, step_size))
    axs[0].set_yticks(np.arange(0, border_size + minor_size, minor_size), minor=True)
    axs[0].set_xticks(np.arange(0, border_size + minor_size, minor_size), minor=True)

    for drone_id in trace_points["drone_id"].unique():
        drone_data = trace_points[trace_points["drone_id"] == drone_id]
        axs[0].plot(
            drone_data["position_x"],
            drone_data["position_y"],
            marker="",
            label=f"Drone {drone_id}",
            linewidth=1,
        )
    bin_size = int(bin_size)
    min_boundary = 0
    max_boundary = border_size
    # Define the grid for the heatmap
    x_bins = np.linspace(min_boundary, max_boundary, bin_size)
    y_bins = np.linspace(min_boundary, max_boundary, bin_size)

    heatmap, xedges, yedges = np.histogram2d(
        data_frame["position_x"], data_frame["position_y"], bins=[x_bins, y_bins]
    )
    heatmap = np.clip(heatmap, 0, 100)

    heatmap = np.flipud(heatmap.T)

    axs[1].imshow(
        heatmap,
        aspect="equal",
        cmap="plasma",
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    )
    # fig.colorbar(im, ax=axs[1], pad=0.001, location=colorbar_location)
    plt.savefig("heatmap_trace.pdf")

    return axs[0], axs[1]


if __name__ == "__main__":
    td = create_dataframe("2024-07-01_01-13-20_Flocking.log")
    od = create_dataframe("2024-07-01_01-13-20_Flocking.log")

    mosaic = """
        fad
        bce
    """
    mosaic = """
        abc
        def
    """
    fig = plt.figure(figsize=(10, 6), layout="constrained")
    ax_dict = fig.subplot_mosaic(
        mosaic,
    )

    image_path = "testbed_screenshot.png"
    ax_dict["a"].imshow(mpimg.imread(image_path))
    ax_dict["a"].axis("off")

    plot_targets_found(td, ax_dict["b"])
    plot_speed(od, ax_dict["d"])

    plot_nearest_neighbor_distance(od, ax_dict["e"])
    fig.legend(loc="outside lower center", ncols=4)

    plot_trace(od, ax_dict["c"], "500")
    im, _ = plot_heatmap(od, ax_dict["f"], "50", "500")
    axins = inset_axes(
        ax_dict["f"],
        width="5%",  # width: 5% of parent_bbox width
        height="50%",  # height: 50%
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax_dict["f"].transAxes,
        borderpad=0.0,
    )
    fig.colorbar(im, cax=axins)
    for label, ax in ax_dict.items():
        # Use ScaledTranslation to put the label
        # - at the top left corner (axes fraction (0, 1)),
        # - offset 20 pixels left and 7 pixels up (offset points (-20, +7)),
        # i.e. just outside the axes.
        new_label = f"({label})"
        ax.text(
            0.5,
            0.0,
            new_label,
            transform=(
                ax.transAxes + ScaledTranslation(-4 / 72, -50 / 72, fig.dpi_scale_trans)
            ),
            fontsize="medium",
            va="bottom",
            fontfamily="serif",
        )
    fig.get_layout_engine().set(w_pad=4 / 72, h_pad=4 / 72, hspace=0.0, wspace=0.0)
    fig.savefig("testbed_results.svg", bbox_inches="tight")

    plt.show()
