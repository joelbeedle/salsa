import pandas as pd
import numpy as np

df = pd.read_csv("../results/results.csv")

grouped = df.groupby(["num_drones", "num_targets"])


def get_stats(group):
    mean_rtf = np.mean(group["rtf"])
    std_dev_rtf = np.std(group["rtf"])
    return pd.Series(
        [mean_rtf, std_dev_rtf], index=["Average RTF", "Standard Deviation"]
    )


results = grouped.apply(get_stats)

print(results)

results.to_csv("table.csv")
