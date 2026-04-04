# Standalone Script
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

if len(sys.argv) < 2:
    print("Usage: python3 analyze_session.py <path_to_csv>")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])
t = df['ros_timestamp'] - df['ros_timestamp'].iloc[0]

fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot 1: FSR vs Target
axs[0].plot(t, df['fsr_force_N'], label='Real FSR', color='blue')
axs[0].plot(t, df['target_force_N'], label='Target', color='red', linestyle='--')
axs[0].set_ylabel('Force (N)')
axs[0].legend()
axs[0].set_title('FSR Force Tracking')

# Plot 2: PID
axs[1].plot(t, df['p_term'], label='P')
axs[1].plot(t, df['i_term'], label='I')
axs[1].plot(t, df['d_term'], label='D')
axs[1].set_ylabel('Command (rad)')
axs[1].legend()

# Plot 3: Servo Position
axs[2].plot(t, df['servo_pos_rad'], label='Servo Pos', color='green')
axs[2].set_ylabel('Pos (rad)')

# Plot 4: Twin Delta
axs[3].plot(t, df['twin_delta_N'], label='Delta (Real - Sim)', color='purple')
axs[3].fill_between(t, -0.5, 0.5, color='gray', alpha=0.2, label='0.5N Threshold')
axs[3].set_ylabel('Error (N)')
axs[3].set_xlabel('Time (s)')

plt.tight_layout()
os.makedirs(os.path.expanduser('~/gripper_logs/analysis'), exist_ok=True)
plt.savefig(os.path.expanduser('~/gripper_logs/analysis/latest_run.png'))
print("Analysis saved to ~/gripper_logs/analysis/")
plt.show()
