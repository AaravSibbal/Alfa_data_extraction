import sys
import pandas as pd
from pymavlink import mavutil

log = mavutil.mavlink_connection(sys.argv[1])
rows = {"IMU": [], "NKF1": []}

while (m := log.recv_match(type=list(rows))) is not None:
    rows[m.get_type()].append(m.to_dict())

imu  = pd.DataFrame(rows["IMU"])[["TimeUS", "GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"]]
nkf1 = pd.DataFrame(rows["NKF1"])[["TimeUS", "Roll", "Pitch", "Yaw"]]

# Find the first moment the EKF has aligned its heading
aligned = nkf1[nkf1["Yaw"] != 0.0]
if aligned.empty:
    sys.exit("Yaw never becomes non-zero in this log; nothing to write.")
start_us = aligned["TimeUS"].iloc[0]

imu_before, nkf1_before = len(imu), len(nkf1)
imu  = imu[imu["TimeUS"] >= start_us].reset_index(drop=True)
nkf1 = nkf1[nkf1["TimeUS"] >= start_us].reset_index(drop=True)

print(f"EKF aligned at TimeUS={start_us} ({start_us/1e6:.1f} s after boot)")
print(f"Dropped {imu_before - len(imu)} IMU rows and {nkf1_before - len(nkf1)} NKF1 rows")

for name, df in [("IMU", imu), ("NKF1", nkf1)]:
    dt_ms = df["TimeUS"].diff().median() / 1000
    print(f"{name}: {len(df)} samples, ~{1000/dt_ms:.1f} Hz (median gap {dt_ms:.2f} ms)")

imu.to_csv("imu.csv", index=False)
nkf1.to_csv("nkf1.csv", index=False)

merged = pd.merge_asof(imu, nkf1, on="TimeUS", direction="nearest")
merged.to_csv("merged.csv", index=False)
print("wrote imu.csv, nkf1.csv, merged.csv")