import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# 原始路径点
update_path1 = [(0,0), (1,1), (5,1), (6,3), (9,6), (13,6), (17,9), (18,16), (18,19), (19,19)]

# 提取 x 和 y 坐标
x, y = zip(*update_path1)

# 生成B样条曲线，s=0表示严格通过所有点
tck, u = splprep([x, y], s=0)

# 生成更密集的插值点
u_fine = np.linspace(0, 1, 100)

# 计算B样条平滑曲线
x_smooth, y_smooth = splev(u_fine, tck)

# 可视化路径
plt.figure(figsize=(8, 6))
plt.plot(x, y, 'ro-', label="原始路径")  # 原始点
plt.plot(x_smooth, y_smooth, 'b-', label="B样条平滑路径")  # 平滑路径
plt.legend()
plt.xlabel("X 坐标")
plt.ylabel("Y 坐标")
plt.title("B样条路径平滑")
plt.grid()
plt.show()
