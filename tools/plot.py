import csv
import matplotlib.pyplot as plt

true_x, true_y, meas_x, meas_y = [], [], [], []

with open("output/run.csv") as f:
    for row in csv.DictReader(f):
        true_x.append(float(row["true_x"]))
        true_y.append(float(row["true_y"]))
        meas_x.append(float(row["meas_x"]))
        meas_y.append(float(row["meas_y"]))

plt.figure(figsize=(10,7))
plt.scatter(meas_x, meas_y, s=6, alpha=0.35, label="GPS measurements", color="tab:orange")
plt.plot(true_x, true_y, linewidth=2, label="Ground truth", color="tab:blue")

plt.axis("equal")
plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("Simulated vessel track with noisy GPS")
plt.legend()
plt.grid(alpha=0.3)
plt.savefig("output/track.png", dpi=150, bbox_inches="tight")
print("Wrote output/track.png")
