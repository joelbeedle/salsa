import pandas as pd
import numpy as np
import sys


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 table.py <filename.csv>")
        sys.exit(1)

    filename = sys.argv[1]

    try:
        df = pd.read_csv(f"../results/{filename}")
    except FileNotFoundError:
        print(f"File not found: {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        sys.exit(1)

    grouped = df.groupby(["num_drones", "num_targets"])

    # Function to calculate mean and standard deviation
    def get_stats(group):
        mean_rtf = np.mean(group["rtf"])
        std_dev_rtf = np.std(group["rtf"])
        return pd.Series(
            [mean_rtf, std_dev_rtf], index=["Average RTF", "Standard Deviation"]
        )

    # Apply the function to each group
    results = grouped.apply(get_stats)

    print(results)

    # Save results to a CSV if you need to export the results
    # results.to_csv('results.csv')


if __name__ == "__main__":
    main()
