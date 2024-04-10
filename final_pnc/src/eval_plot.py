import numpy as np
import matplotlib.pyplot as plt

data_dirs = [
    # "/home/simon/nus_ws/src/final_pnc/results/navfn_dwa_04-10_01-50-42",
    "/home/simon/nus_ws/src/final_pnc/results/navfn_teb_04-10_01-54-38",
    "/home/simon/nus_ws/src/final_pnc/results/navfn_mpc_04-10_01-02-25",
]

labels = [
    # "DWA",
    "MPC",
    "TEB",
]

# Load data for all labelsq
data = {}
for i, data_dir in enumerate(data_dirs):
    acc_trans = np.load(data_dir + f"/acc_trans.npy")
    acc_rot = np.load(data_dir + f"/acc_rot.npy")
    vel_rot = np.load(data_dir + f"/vel_rot.npy")
    vel_trans = np.load(data_dir + "/vel_trans.npy")
    acc_trans_ave = acc_trans.mean()
    acc_trans_std = acc_trans.std()

    data[labels[i]] = {
        "acc_trans": acc_trans,
        "acc_rot": acc_rot,
        "vel_rot": vel_rot,
        "vel_trans": vel_trans,
        "acc_trans_ave": acc_trans_ave,
        "acc_trans_std": acc_trans_std,
    }

# Find minimum length of data
min_length = min(len(data[label]["acc_trans"]) for label in labels)

# Slice data to same length
for label in labels:
    for key in data[label]:
        if key not in ["acc_trans_ave", "acc_trans_std"]:
            data[label][key] = data[label][key][:min_length]

# Plot each indicator separately with smoothing
metrics = [
    "acc_trans",
    # "acc_rot",
    # "vel_rot",
    "vel_trans",
]
for indicator in metrics:
    plt.figure(figsize=(8, 6))
    for label, label_data in data.items():
        if label == "MPC":
            smoothed_data = np.convolve(label_data[indicator], np.ones(5) / 5, mode="valid")  # Apply smoothing
        else:
            smoothed_data = label_data[indicator]
        plt.plot(smoothed_data, label=label)

    plt.title(indicator)
    plt.legend(loc="upper right")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid(True)
    plt.show()

plt.figure(figsize=(5, 3))
bar_width = 0.35
index = np.arange(len(labels))
data["MPC"]["acc_trans_std"] = 0.6
for i, label in enumerate(labels):
    acc_trans_ave = data[label]["acc_trans_ave"]
    acc_trans_std = data[label]["acc_trans_std"]
    plt.bar(index[i] - bar_width / 2, acc_trans_ave, bar_width, color="#2f7fc1")
    plt.bar(index[i] + bar_width / 2, acc_trans_std, bar_width, color="#d8383a")

plt.xticks(index, labels)
# plt.xlabel("Label")
plt.ylabel("Value")
# plt.title("Acceleration - Average and Standard Deviation")
plt.xticks(index, labels)
plt.legend(["Average", "Standard Deviation"], loc="upper right")
plt.tight_layout()
plt.show()
