import pandas as pd
import matplotlib.pyplot as plt
path = input("Type CSV Path: ")
# 读取 CSV
df = pd.read_csv('/home/luo/Project/Horse/Dog_parallel/src/dog_trajectory/test/test.csv')

# 绘制位置轨迹 (x-z 平面)
plt.figure(figsize=(12, 5))

ax1 = plt.subplot(1, 2, 1)
ax1.plot(df['x'], df['z'], label='Trajectory (x-z)', color='blue')
ax1.scatter(df['x'].iloc[0], df['z'].iloc[0], color='green', label='Start', s=50)
ax1.scatter(df['x'].iloc[-1], df['z'].iloc[-1], color='red', label='End', s=50)
ax1.set_xlabel('X Position')
ax1.set_ylabel('Z Position')
ax1.set_title('Position Trajectory')
ax1.legend()
ax1.invert_yaxis()
ax1.invert_xaxis()
ax1.grid(True)

# 绘制速度 (vx, vz)
ax2 = plt.subplot(1, 2, 2)
ax2.plot(df['time'], df['vx'], label='Vx', color='orange')
ax2.plot(df['time'], df['vz'], label='Vz', color='purple')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity')
ax2.set_title('Velocity over Time')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()