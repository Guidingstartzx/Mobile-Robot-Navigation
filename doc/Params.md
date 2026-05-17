# 关键参数
- **`minimum_turning_radius: 0.60`（最小转弯半径）**
	这是由小车的轴距（$L=0.56\text{m}$）和前轮最大转向角（$\delta_{max}=0.75\text{rad}$）决定的物理极限。公式为 $R = L / \tan(\delta)$。**软件参数不能打破物理定律**，即使调小也不能真正实现
- `inflation_radius`（代价地图膨胀半径）
