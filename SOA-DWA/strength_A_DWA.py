import numpy as np
import heapq
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from scipy.interpolate import interp1d
from scipy.interpolate import splprep, splev
import common_A

# 设置全局字体
plt.rcParams['font.sans-serif'] = ['SimSun']  # 使用宋体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 生成带有手动指定障碍物的栅格地图
def generate_grid_with_obstacles(rows, cols, obstacles):
    grid = np.zeros((rows, cols))

    # 根据给定的障碍物坐标设置障碍物
    for obs in obstacles:
        x, y = obs
        if 0 <= x < rows and 0 <= y < cols:
            grid[x][y] = 2  # 设置障碍物

    return grid

# 绘制栅格地图和路径，添加动态障碍物参数
def draw_grid_with_path(grid, path, start, end, dynamic_obstacles=None):
    fig, ax = plt.subplots(figsize=(7,7))

    # 显示栅格地图，并禁用插值，确保每个格子独立显示
    ax.imshow(grid, cmap='binary', origin='lower', interpolation='none')

    ax.set_xticks(np.arange(0, grid.shape[1], 1))  
    ax.set_yticks(np.arange(0, grid.shape[0], 1))  
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)  

    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)

    # 绘制路径，将路径上的点连接起来，设置为浅蓝色虚线
    # if path:
    #    # 提取路径点的 x 和 y 坐标
    #     y, x = zip(*path)

    #     # 生成B样条曲线
    #     tck, u = splprep([x, y], s=0.5)  # s=0 保证严格通过所有点
    #     u_fine = np.linspace(0, 1, 50)  # 生成更密集的插值点

    #     # 计算B样条平滑曲线
    #     x_smooth, y_smooth = splev(u_fine, tck)

    #     # 绘制原始路径（红点表示原始路径）
    #     # ax.plot(x, y, 'ro-', label="原始路径")  
    #     # ax.plot(x_smooth, y_smooth, color='#0080FF', linestyle='--', linewidth=2)  # 浅蓝色虚线
    #     # ax.plot(x_smooth, y_smooth, color='#0080FF', linestyle='--', linewidth=2)  # 浅蓝色虚线
    #     ax.plot(x, y, color='red',  linewidth=2)  # 浅蓝色虚线


    # 绘制动态障碍物（黄色方块）
    # if dynamic_obstacles:
    #     dynamic_added = False  # 确保图例只添加一次
    #     for obs in dynamic_obstacles:
    #         x, y = obs
    #         # 检查坐标是否在网格范围内
    #         if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
    #             if not dynamic_added:
    #                 ax.scatter(y, x, color='yellow', s=150, marker='s', 
    #                            edgecolor='black', linewidth=0.5, zorder=3,)
    #                 dynamic_added = True
    #             else:
    #                 ax.scatter(y, x, color='yellow', s=150, marker='s', 
    #                            edgecolor='black', linewidth=0.5, zorder=3)

    # 绘制起点和终点
    ax.scatter(start[1], start[0], color='green', s=100,  marker='*')  
    ax.scatter(end[1], end[0], color='blue', s=100 , marker='*')  

    # 在 (0, 13) 栅格上加一个向左的箭头
    # ax.annotate('', xy=(11, 0), xytext=(12, 0),
    #             arrowprops=dict(facecolor='red', shrink=0.05, width=2, headwidth=8, headlength=10),
    #             fontsize=12, color='red')  
    # ax.annotate('', xy=(22, 9.5), xytext=(23, 9.5),
    #             arrowprops=dict(facecolor='red', shrink=0.05, width=2, headwidth=8, headlength=10),
    #             fontsize=12, color='red') 

    # 添加图例，调整位置防止遮挡
    ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1))



    a = np.array(path[0])
    b = np.array(path[1])
    

    spl = interp1d(x_control, y_control, kind='cubic', fill_value="extrapolate") 

    x_smooth = np.linspace(x_control.min(), x_control.max(), 500)
    y_smooth = spl(x_smooth)


    # 绘制结果
    plt.plot(x_smooth, y_smooth, label="平滑曲线", color="red")
    # plt.scatter(x_control5, y_control5, color="red", label="控制点")
        

    # plt.title("SOA-DWA算法的路径规划", fontsize=16)
    # plt.xlabel("X (单位：km)", fontsize=12)
    # plt.ylabel("Y (单位：km)", fontsize=12)
    
    plt.show()


def draw_grid_with_path_smooth(grid, path, start, end):
    fig, ax = plt.subplots()

    # 显示栅格地图，并禁用插值，确保每个格子独立显示
    ax.imshow(grid, cmap='binary', origin='lower', interpolation='none')

    ax.set_xticks(np.arange(0, grid.shape[1], 1))  
    ax.set_yticks(np.arange(0, grid.shape[0], 1))  
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)  

    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)

    # 绘制路径，将路径上的点连接起来，设置为浅蓝色虚线
    if path:
       # 提取路径点的 x 和 y 坐标
        y, x = zip(*path)

        # 生成B样条曲线
        tck, u = splprep([x, y], s=0.5)  # s=0 保证严格通过所有点
        u_fine = np.linspace(0, 1, 50)  # 生成更密集的插值点

        # 计算B样条平滑曲线
        x_smooth, y_smooth = splev(u_fine, tck)

        # 绘制原始路径（红点表示原始路径）
        # ax.plot(x, y, 'ro-', label="原始路径")  
        # ax.plot(x_smooth, y_smooth, color='#0080FF', linestyle='--', linewidth=2)  # 浅蓝色虚线
        ax.plot(x_smooth, y_smooth, color='red', linewidth=2)  # 浅蓝色虚线


    ax.scatter(start[1], start[0], color='green', s=100, label="Start", marker='*')  
    ax.scatter(end[1], end[0], color='blue', s=100, label="End", marker='*')  

    plt.title("基于改进A*算法的路径规划", fontsize=16)
    plt.xlabel("X (单位：km)", fontsize=12)
    plt.ylabel("Y (单位：km)", fontsize=12)
    
    plt.show()

# 主函数
if __name__ == '__main__':
    rows1, cols1 = 20, 20
    rows2, cols2 = 30, 30

    
    
    obstacles2 = [
        (0, 24),(0, 25),
        (1, 13),(1, 14),(1, 15),(1, 19),(1, 24),(1, 25),
        (2, 8),(2, 13),
        (3, 2),(3, 3),(3, 7),(3, 8),(3, 24),
        (4, 19),(4, 20),(4, 24),
        (5, 19),(5, 20),(5, 28),(5, 29),
        (6, 0),(6, 15),(6, 28),(6, 29),
        (7, 0),(7, 4),(7, 5),(7, 6),(7, 15),(7, 17),(7, 18),(7, 22),(7, 23),(7, 24),(7, 25),(7, 26),(7, 27),
        (8, 0),(8, 1),(8, 15),(8, 23),(8, 24),(8, 25),
        (9, 0),(9, 1),(9, 7),(9, 8),(9, 9),
        (10, 9),
        (11, 5),(11, 6),(11, 25),(11, 26),
        (12, 3),(12, 4),(12, 5),(12, 6),(12, 18),(12, 19),(12, 20),(12, 23),(12, 24),(12, 25),(12, 26),
        (13, 19),(13, 20),(13, 25),(13, 26),
        (14, 9),(14, 10),(14, 11),(14, 12),(14, 13),(14, 14),(14, 15),(14, 16),(14, 17),(14, 18),(14, 19),(14, 20),
        (15, 5),(15, 10),(15, 11),(15, 12),
        (16, 4),(16, 5),(16, 11),
        (17, 3),(17, 4),(17, 5),(17, 29),
        (18, 3),(18, 10),(18, 11),(18, 12),(18, 13),(18, 25),(18, 29),
        (19, 9),(19, 10),(19, 11),(19, 12),(19, 13),(19, 14),(19, 15),(19, 16),(19, 17),(19, 18),(19, 19),(19, 20),(19, 25),(19, 26),(19, 29),
        (20, 25),
        (22, 16),(22, 29),
        (23, 2),(23, 3),(23, 4),(23, 16),(23, 22),(23, 25),(23, 29),
        (24, 14),(24, 22),(24, 25),(24, 26),(24, 29),
        (25, 13),(25, 14),
        (26, 6),(26, 7),(26, 19),(26, 20),
        (27, 6),(27, 12),(27, 15),(27, 16),(27, 20),
        (28, 11),(28, 12),(28, 23),(28, 24),
        (29, 10),(29, 11)
    ]

    obstacles3 = [
        (0, 4),(0, 5),(0, 6),(0, 13),(0, 14),(0, 15),(0, 22),(0, 23),(0, 24),(0, 25),
        (1, 11),(1, 12),(1, 13),(1, 14),(1, 15),(1, 19),(1, 24),(1, 25),
        (2, 2),(2, 3),(2, 8),(2, 13),(2, 15),(2, 16),(2, 18),(2, 19),(2, 22),(2, 24),
        (3, 2),(3, 3),(3, 7),(3, 8),(3, 13),(3, 14),(3, 15),(3, 20),(3, 21),(3, 24),
        (4, 8),(4, 9),(4, 13),(4, 14),(4, 15),(4, 19),(4, 20),(4, 24),
        (5, 4),(5, 5),(5, 6),(5, 19),(5, 20),(5, 28),(5, 29),
        (6, 0),(6, 11),(6, 12),(6, 15),(6, 28),(6, 29),
        (7, 0),(7, 4),(7, 5),(7, 6),(7, 11),(7, 12),(7, 15),(7, 17),(7, 18),(7, 22),(7, 23),(7, 24),(7, 25),(7, 26),(7, 27),
        (8, 0),(8, 1),(8, 11),(8, 12),(8, 15),(8, 23),(8, 24),(8, 25),
        (9, 0),(9, 1),(9, 7),(9, 8),(9, 9),(9, 15),(9, 16),(9, 17),(9, 18),
        (10, 9),(10, 15),(10, 16),(10, 17),(10, 18),(10, 25),(10, 26),(10, 27),(10, 28),
        (11, 5),(11, 6),(11, 13),(11, 14),(11, 25),(11, 26),
        (12, 0),(12, 1),(12, 3),(12, 4),(12, 5),(12, 6),(12, 11),(12, 13),(12, 14),(12, 18),(12, 19),(12, 20),(12, 23),(12, 24),(12, 25),(12, 26),
        (13, 0),(13, 1),(13, 11),(13, 18),(13, 19),(13, 20),(13, 25),(13, 26),
        (14, 0),(14, 1),(14, 9),(14, 10),(14, 11),(14, 12),(14, 13),(14, 14),(14, 15),(14, 16),(14, 17),(14, 18),(14, 19),(14, 20),
        (15, 5),(15, 10),(15, 11),(15, 12),(15, 25),(15, 26),(15, 27),(15, 28),
        (16, 4),(16, 5),(16, 11),(16, 18),(16, 19),(16, 22),(16, 23),(16, 25),(16, 26),(16, 27),(16, 28),
        (17, 3),(17, 4),(17, 5),(17, 18),(17, 19),(17, 22),(17, 23),(17, 29),
        (18, 3),(18, 10),(18, 11),(18, 12),(18, 13),(18, 22),(18, 23),(18, 25),(18, 29),
        (19, 9),(19, 10),(19, 11),(19, 12),(19, 13),(19, 14),(19, 15),(19, 16),(19, 17),(19, 18),(19, 19),(19, 20),(19, 25),(19, 26),(19, 29),
        (20, 2),(20, 3),(20, 4),(20, 5),(20, 25),
        (21, 2),(21, 3),(21, 4),(21, 5),(21, 21),(21, 22),(21, 23),(21, 24),(21, 27),(21, 28),(21, 29),
        (22, 7),(22, 8),(22, 9),(22, 12),(22, 13),(22, 16),(22, 29),
        (23, 2),(23, 3),(23, 4),(23, 7),(23, 8),(23, 9),(23, 12),(23, 13),(23, 16),(23, 22),(23, 25),(23, 29),
        (24, 14),(24, 17),(24, 18),(24, 22),(24, 25),(24, 26),(24, 29),
        (25, 13),(25, 14),(25, 17),(25, 18),
        (26, 6),(26, 7),
        (27, 0),(27, 2),(27, 2),(27, 6),(27, 12),(27, 15),(27, 16),(27, 26),(27, 27),
        (28, 0),(28, 1),(28, 2),(28, 11),(28, 12),(28, 23),(28, 24),(28, 26),(28, 27),(28, 19),(28, 20),
        (29, 0),(29, 1),(29, 2),(29, 10),(29, 11),(29, 26),(29, 27),(29, 20),
    ]
    
    grid3 = generate_grid_with_obstacles(rows2, cols2, obstacles3)
    start2 = (0, 0)
    end2 = (29, 29)
    update_path2 = [(0,0),(0,2),(0,4),(0,6),(0,8),(0,10),(0,12),(0,16),(0,20),(5,21),(8,21),(9,22),(9,28),(12,28),(16,28),(18,28),(20,28),(22,28),(25.5,28.5),(29,29)]

    grid2 = generate_grid_with_obstacles(rows2, cols2, obstacles2)
    start2 = (0, 0)
    end2 = (29, 29)
    update_path3 = [(0,0),(4,1),(8,3),(10,7),(12,8),(14,8),(16,8),(18,8),(20,8),(20,10),(20,12),(20,16),(20,18),(22,20),(22,23),(22,25),(22,26),(22,27),(25,28),(29,29)]

    path = common_A.a_star(grid3, start2, end2)

    if update_path3:
        print("A*算法找到路径！")
        print(update_path3)

        # 手动指定新增障碍物的位置
        new_obstacles = [(4, 2), (4, 3), (16, 6), (16, 7), (17, 6), (17, 7), (21, 14), (21, 15), (21, 16), (22, 28), (23, 28), (24, 28),]  # 新增障碍物
        new_obstacles2 = [(0, 11), (0, 12), (1, 11), (1, 12), (9, 19), (9, 20), (10, 19), (10, 20), (7, 28), (7, 29), (8, 28), (8, 29), (9, 28), (9, 29)]
        new_obstacles3 = [(0, 11), (0, 12), (1, 11), (1, 12), (2, 16), (3, 16), (4, 16), (2, 17), (3, 17), (4, 17), (2, 18), (3, 18), (4, 18), (9, 28), (10, 28), (9, 29), (10, 29), (15, 28), (15, 29), (16, 28), (16, 29)]
        new_obstacles4 = [(8, 5), (8, 6), (9, 5), (9, 6), (20, 13), (20, 14),(20, 15), (25, 27),(25, 28),(25, 29)]
        new_dynamic_obstacles1 = [(0, 16), (1, 16), (2, 16), (15, 24), (16, 24), (15, 23), (16, 23)]
        new_dynamic_obstacles2 = [(0, 13), (0, 14), (0, 15),(9, 24), (10, 24), (9, 25), (10, 25)]
        new_dynamic_obstacles3 = [(1, 4), (1, 5), (1, 6),(22, 22), (22, 23), (22, 24)]
        for obs in new_obstacles:
            x, y = obs
            if 0 <= x < rows2 and 0 <= y < cols2:
                grid3[x][y] = 1
            else:
                print(f"障碍物坐标 {obs} 超出范围，忽略！")
        
        # for obs in new_dynamic_obstacles:
        #     x, y = obs
        #     if 0 <= x < rows2 and 0 <= y < cols2:
        #         grid2[x][y] = 1
        #     else:
        #         print(f"障碍物坐标 {obs} 超出范围，忽略！")

        # print(f"新增的静态障碍物坐标: {new_obstacles}")
       
    # draw_grid_with_path(grid2, update_path2, start2, end2)
    draw_grid_with_path(grid3, update_path3, start2, end2, dynamic_obstacles= new_obstacles)
    # draw_grid_with_path_smooth(grid3, update_path3, start2, end2)
    if draw_grid_with_path:
            print("路径找到！")
    else:
        print("没有找到路径")    
       
