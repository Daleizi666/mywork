import numpy as np
import heapq
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

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
            grid[x][y] = 1  # 设置障碍物

    # 确保起点和终点没有障碍物
    grid[0][0] = 0  # 起点
    grid[rows-1][cols-1] = 0  # 终点

    return grid

def is_clear_path(grid, start, end):
    x1, y1 = start
    x2, y2 = end
    
    # 检查起点和终点是否越界
    if not (0 <= x1 < len(grid) and 0 <= y1 < len(grid[0]) and
            0 <= x2 < len(grid) and 0 <= y2 < len(grid[0])):
        print("Start or end is out of grid bounds")
        return False
    
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        # 检查当前格子是否有障碍物
        if grid[x1][y1] == 1:  # 1表示障碍物
            print(f"Obstacle detected at {x1, y1}")
            return False
        
        # 检查是否到达终点
        if x1 == x2 and y1 == y2:
            break
        
        e2 = 2 * err
        
        # 判断是否越过横向边界
        if e2 > -dy:
            err -= dy
            x1 += sx
        
        # 判断是否越过纵向边界
        if e2 < dx:
            err += dx
            y1 += sy
        
        # 在每次移动后都检查当前位置
        if grid[x1][y1] == 1:  # 1表示障碍物
            print(f"Obstacle detected at {x1, y1}")
            return False

    return True





# 曼哈顿距离（启发式函数）
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# 计算两点之间的欧几里得距离
def euclidean_distance(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def calculate_path_length(path, start_idx, end_idx):
    total_length = 0
    for i in range(start_idx, end_idx):
        total_length += heuristic(path[i], path[i + 1])
    return total_length


# A*算法
def a_star(grid, start, end):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, end), 0, start))  # (f, g, position)
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
            
            # 检查非相邻节点之间的连接距离，并更新路径
            updated_path = [path[0]]
            for i in range(1, len(path) - 1):
                # 检查路径上的非相邻节点对之间的连接
                if euclidean_distance(updated_path[-1], path[i]) < euclidean_distance(updated_path[-1], path[i+1]):
                    updated_path.append(path[i])  # 添加当前节点
                else:
                    # 更新路径
                    updated_path.append(path[i+1])
            updated_path.append(path[-1])

            return updated_path

        closed_list.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]
                and grid[neighbor[0]][neighbor[1]] == 0 and neighbor not in closed_list):

                tentative_g_score = g + 1  # 假设每个移动的成本为1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score, tentative_g_score, neighbor))

    return None  # 如果没有找到路径，返回None

def update_path_clear_nodes(grid, path):
    update_path = [path[0]]
    current_node = path[0]
    i = 0
    while i < len(path) - 1:
        found = False
        for j in range(i + 2, len(path)):
            # 计算从当前节点到非相邻节点的直连距离
            direct_distance = euclidean_distance(current_node, path[j])
            # 计算从当前节点到非相邻节点的原路径长度（中间节点之间的路径长度）
            original_path_distance = calculate_path_length(path, i, j)  # 传入正确的索引范围
            print(i, j, direct_distance, original_path_distance,current_node,path[j])
            
            # 如果直连路径短且没有障碍物，则更新路径
            if direct_distance < original_path_distance and is_clear_path(grid, current_node, path[j]):
                update_path.append(path[j])
                current_node = path[j]  # 更新上一个添加的节点
                i = j  # 更新当前位置
                found = True
                break  # 找到后退出内层循环
        
        # 如果没有找到合适的节点，则继续往下找
        if not found:
            update_path.append(path[i])
            i += 1  # 如果找不到符合条件的节点，继续向后查找
            current_node = path[i]
    
    return update_path




def draw_grid_with_path(grid, path, start, end):
    fig, ax = plt.subplots(figsize=(7,7))

    # 显示栅格地图，并禁用插值，确保每个格子独立显示
    ax.imshow(grid, cmap='binary', origin='lower', interpolation='none')

    ax.set_xticks(np.arange(0, grid.shape[1], 1))  
    ax.set_yticks(np.arange(0, grid.shape[0], 1))  
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)  

    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)

    # 绘制路径，将路径上的点连接起来，设置为浅蓝色虚线
    if path:
        path_x = [p[1] for p in path]  
        path_y = [p[0] for p in path]  
        ax.plot(path_x, path_y, color='#0080FF', linestyle='--', linewidth=2)  # 浅蓝色虚线

    ax.scatter(start[1], start[0], color='green', s=100, label="Start", marker='*')  
    ax.scatter(end[1], end[0], color='blue', s=100, label="End", marker='*')  

    # plt.title("基于改进A*算法的路径规划", fontsize=16)
    # plt.xlabel("X (单位：km)", fontsize=12)
    # plt.ylabel("Y (单位：km)", fontsize=12)
    
    plt.show()

def draw_grid_with_path_smooth(grid, path, start, end):
    fig, ax = plt.subplots(figsize=(7,7))

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

    # plt.title("基于改进A*算法的路径规划", fontsize=16)
    # plt.xlabel("X (单位：km)", fontsize=12)
    # plt.ylabel("Y (单位：km)", fontsize=12)
    
    plt.show()


# 主函数
if __name__ == '__main__':
    rows1, cols1 = 20, 20
    rows2, cols2 = 30, 30
    obstacles1 = [
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
        (10, 10),(10, 12),(10, 13),(10, 14),(10, 16),(10, 19),
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

    grid1 = generate_grid_with_obstacles(rows1, cols1, obstacles1)
    start1 = (0, 0)
    end1 = (19, 19)
    path1 = a_star(grid1, start1, end1)

    grid2 = generate_grid_with_obstacles(rows2, cols2, obstacles2)
    start2 = (0, 0)
    end2 = (29, 29)
    path2 = a_star(grid2, start2, end2)
   
    
    grid3 = generate_grid_with_obstacles(rows2, cols2, obstacles3)
    start2 = (0, 0)
    end2 = (29, 29)
    path3 = a_star(grid3, start2, end2)
    

    try:
        option = int(input("Enter 1 to run tabu_search_test(100), 2 to run preannealing_test(100): "))
        if option == 1:
            draw_grid_with_path(grid1, path1, start1, end1)
            draw_grid_with_path(grid1, update_path1, start1, end1)
            draw_grid_with_path_smooth(grid1, update_path1, start1, end1)
            if path1:
                print("路径找到！")
            else:
                print("没有找到路径")
        elif option == 2:
            draw_grid_with_path(grid2, path2, start2, end2)
            draw_grid_with_path(grid2, update_path2, start2, end2)
            draw_grid_with_path_smooth(grid2, update_path2, start2, end2)
            if path1:
                print("路径找到！")
            else:
                print("没有找到路径")    
        elif option == 3:
            draw_grid_with_path(grid3, path3, start2, end2)
            draw_grid_with_path(grid3, update_path3, start2, end2)
            draw_grid_with_path_smooth(grid3, update_path3, start2, end2)
            if path1:
                print("路径找到！")
            else:
                print("没有找到路径")       
    except ValueError as e:
        print(e)
