import numpy as np
import heapq
import matplotlib.pyplot as plt
import time

# 设置全局字体
plt.rcParams['font.sans-serif'] = ['SimSun']  # 使用宋体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 生成带有手动指定障碍物的栅格地图
def generate_grid_with_obstacles(rows, cols, obstacles):
    grid = np.zeros((rows, cols))
    for obs in obstacles:
        x, y = obs
        if 0 <= x < rows and 0 <= y < cols:
            grid[x][y] = 1  # 设置障碍物
    grid[0][0] = 0  # 起点
    grid[rows-1][cols-1] = 0  # 终点
    return grid

# 曼哈顿距离（启发式函数）
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A*算法
def a_star(grid, start, end):
    start_time = time.time()
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, end), 0, start))
    closed_list = set()
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, g, current = heapq.heappop(open_list)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            end_time = time.time()
            print(f"A*算法耗时: {end_time - start_time:.4f}秒")
            return path

        closed_list.add(current)
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]
                and grid[neighbor[0]][neighbor[1]] == 0 and neighbor not in closed_list):
                tentative_g_score = g + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score, tentative_g_score, neighbor))
    
    return None


# 绘制栅格地图和路径
def draw_grid_with_path(grid, path, start, end):
    fig, ax = plt.subplots(figsize=(7,7))
    ax.imshow(grid, cmap='binary', origin='lower', interpolation='none')
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)

    if path:
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        ax.plot(path_x, path_y, color='red', linewidth=2)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i][j] == 1:
                ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, facecolor='black', edgecolor='black'))

    # for i in range(grid.shape[0]):
    #     for j in range(grid.shape[1]):
    #         if grid[i][j] == 2:
    #             ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, facecolor='gray', edgecolor='black'))

    ax.scatter(start[1], start[0], color='green', s=100, label="Start", marker='*')
    ax.scatter(end[1], end[0], color='blue', s=100, label="End", marker='*')
    # plt.xlabel("X (单位：km)", fontsize=12)
    # plt.ylabel("Y (单位：km)", fontsize=12)
    plt.show()

# 主函数
if __name__ == '__main__':
    rows, cols = 20, 20
    obstacles = [
        (0, 2),(0, 4),(0, 5),(0, 6),(0, 14),(0, 18),(0, 19),
        (1, 6),(1, 7),(1, 8),(1, 10),(1, 18),
        (2, 4),(2, 9),(2, 10),(2, 15),(2, 17),
        (3, 0),(3, 4),(3, 11),(3, 12),(3, 15),
        (4, 10),(4, 18),
        (5, 10),(5, 11),(5, 14),(5, 19),
        (6, 0),(6, 1),(6, 2),
        (7, 1),(7, 2),(7, 3),(7, 4),(7, 5),(7, 9),(7, 10),(7, 16),
        (8, 4),(8, 5),(8, 6),(8, 18),
        (9, 3),(9, 6),(9, 11),
        (10, 12),(10, 13),(10, 14),(10, 16),(10, 19),
        (11, 0),(11, 2),(11, 16),
        (12, 3),(12, 6),(12, 10),
        (13, 10),(13, 16),
        (14, 15),(14, 17),(14, 19),
        (15, 4),(15, 13),(15, 14),
        (16, 3),(16, 7),(16, 10),
        (17, 4),(17, 9),
        (18, 1),(18, 9),(18, 11),(18, 12),(18, 14),(18, 19),
        (19, 8),(19, 13),
    ]
    
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
        (26, 6),(26, 7),(26, 19),(26, 20),
        (27, 0),(27, 2),(27, 2),(27, 6),(27, 12),(27, 15),(27, 16),(27, 20),(27, 26),(27, 27),
        (28, 0),(28, 1),(28, 2),(28, 11),(28, 12),(28, 23),(28, 24),(28, 26),(28, 27),
        (29, 0),(29, 1),(29, 2),(29, 10),(29, 11),(29, 26),(29, 27),
    ]
    
    grid = generate_grid_with_obstacles(rows, cols, obstacles)
    start = (0, 0)
    end = (19, 19)

    grid2 = generate_grid_with_obstacles(30, 30, obstacles2)
    start = (0, 0)
    end = (29, 29)

    grid3 = generate_grid_with_obstacles(30, 30, obstacles3)
    start = (0, 0)
    end = (29, 29)
    
    # 运行A*算法
    path = a_star(grid2, start, end)
    
    if path:
        print("A*算法找到路径！")
        print(path)

        # 手动指定新增障碍物的位置
        new_obstacles = [(4, 2), (4, 3), (16, 6), (16, 7), (17, 6), (17, 7), (21, 14), (21, 15), (21, 16), (22, 28), (23, 28), (24, 28)]  # 新增障碍物
        for obs in new_obstacles:
            x, y = obs
            if 0 <= x < rows and 0 <= y < cols:
                grid[x][y] = 2
            else:
                print(f"障碍物坐标 {obs} 超出范围，忽略！")

        print(f"新增的静态障碍物坐标: {new_obstacles}")


        # 绘制栅格地图和路径
        draw_grid_with_path(grid2, path, start, end)
    else:
        print("A*算法未找到路径！")