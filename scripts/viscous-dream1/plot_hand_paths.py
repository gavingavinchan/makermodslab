"""Static scientific comparison plot; uses the machine's matplotlib installation."""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ["TMPDIR"] = str(HERE / ".cache/tmp")
os.environ["MPLCONFIGDIR"] = str(HERE / ".cache/matplotlib")
sys.pycache_prefix = str(HERE / ".cache/pycache-plot")
import json
import numpy as np


def main():
    plot_data = json.loads((HERE / "runs/hand_path_plot_data.json").read_text())
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(15, 9), layout="constrained")
    for column, (name, data) in enumerate(plot_data.items()):
        trajectory = data["trajectory"]
        ax = fig.add_subplot(2, 3, column + 1, projection="3d")
        requested = np.array([t["tracking"]["left"]["requested_xyz"] for t in trajectory])
        command = np.array([t["tracking"]["left"]["fk_xyz"] for t in trajectory])
        measured = np.asarray(data["measured_left_xyz"])
        apple = np.array([t["apple_base_before_step"]["left"] for t in trajectory])
        for points, label, color in ((requested, "Requested", "#1565c0"), (command, "Command FK", "#ef6c00"),
                                      (measured, "Measured FK", "#388e3c"), (apple, "Apple", "#c62828")):
            ax.plot(*points.T, label=label, color=color, linewidth=1, alpha=.85)
        ax.scatter(*apple[0], color="#c62828", s=40)
        ax.set(xlabel="Base x (m)", ylabel="Base y (m)", zlabel="Base z (m)",
               xlim=(0, .4), ylim=(-.4, .1), zlim=(-.05, .35), title=name.replace("_", " "))
        if column == 0:
            ax.legend(fontsize=8)
        ax = fig.add_subplot(2, 3, column + 4)
        times = np.arange(len(trajectory)) / 20
        ax.plot(times, np.linalg.norm(command-requested, axis=1)*100, label="Command error", color="#ef6c00")
        ax.plot(times, np.linalg.norm(measured-requested, axis=1)*100, label="Measured error", color="#388e3c", alpha=.8)
        ax.plot(times, np.linalg.norm(apple-requested, axis=1)*100, label="Request to apple", color="#1565c0", alpha=.8)
        ax.set(xlabel="Simulated action time (s)", ylabel="Left hand distance (cm)", ylim=(0, 45))
        ax.grid(alpha=.2)
        if column == 0:
            ax.legend(fontsize=8)
    fig.savefig("runs/hand_path_comparison.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
