
import pandas as pd
import matplotlib.pyplot as plt


filename = "mpu_data_kalman_output.csv"  
time_col = "time"


df = pd.read_csv(filename)
print(f"Loaded {len(df)} samples from {filename}")


cols = df.columns.tolist()
print("Columns found:", cols)

roll_col = None
pitch_col = None

if "roll_filtered" in cols and "pitch_filtered" in cols:
    roll_col = "roll_filtered"
    pitch_col = "pitch_filtered"
    title = " Raw Roll & Pitch"

else:
    print(" No roll/pitch columns found. ")
    exit()


plt.figure(figsize=(10, 6))
plt.plot(df[time_col], df[roll_col], label="Roll", linewidth=1.8)
plt.plot(df[time_col], df[pitch_col], label="Pitch", linewidth=1.8)
plt.title(title)
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()
plt.show()
