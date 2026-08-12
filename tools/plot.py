import csv
import matplotlib.pyplot as plt

true_x, true_y, meas_x, meas_y, est_x, est_y = [], [], [], [], [], []

with open("output/run.csv") as f:
    for row in csv.DictReader(f):
        true_x.append(float(row["true_x"]))
        true_y.append(float(row["true_y"]))
        meas_x.append(float(row["meas_x"]))
        meas_y.append(float(row["meas_y"]))
        est_x.append(float(row["est_x"]))
        est_y.append(float(row["est_y"]))

plt.figure(figsize=(10,7))
plt.scatter(meas_x, meas_y, s=6, alpha=0.35, label="GPS measurements", color="tab:orange")
plt.plot(true_x, true_y, linewidth=2, label="Ground truth", color="tab:blue")
plt.plot(est_x, est_y, linewidth=2, label="Kalman estimate", color="tab:green", linestyle="--")

plt.axis("equal")
plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("Vessel track: noisy GPS vs. Kalman estimate")
plt.legend()
plt.grid(alpha=0.3)
plt.savefig("output/track.png", dpi=150, bbox_inches="tight")
print("Wrote output/track.png")

#Error Summary
import math
gps_err = [math.hypot(mx - tx, my - ty) for mx, my, tx, ty in zip(meas_x, meas_y, true_x, true_y)]
est_err = [math.hypot(ex - tx, ey - ty) for ex, ey, tx, ty in zip(est_x, est_y, true_x, true_y)]

def rmse(errs):
    return math.sqrt(sum(e * e for e in errs) / len(errs))

#skip the first 2s while the initial covariance collapses
warmup = 20
print(f"RMSE raw GPS:   {rmse(gps_err[warmup:]):.2f} m")
print(f"RMSE filtered:  {rmse(est_err[warmup:]):.2f} m")
print(f"Improvement:    {rmse(gps_err[warmup:]) / rmse(est_err[warmup:]):.1f}x")


