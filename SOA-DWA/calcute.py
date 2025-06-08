import math

def calculate_angle(A, B, C):
    # 向量 BA 和 BC
    BA = (A[0] - B[0], A[1] - B[1])
    BC = (C[0] - B[0], C[1] - B[1])
    
    # 向量 BA 和 BC 的点积
    dot_product = BA[0] * BC[0] + BA[1] * BC[1]
    
    # 向量 BA 和 BC 的模长
    magnitude_BA = math.sqrt(BA[0]**2 + BA[1]**2)
    magnitude_BC = math.sqrt(BC[0]**2 + BC[1]**2)
    
    # 计算夹角的余弦值
    cos_theta = dot_product / (magnitude_BA * magnitude_BC)
    
    # 计算角度（弧度制）
    angle_rad = math.acos(cos_theta)
    
    # 转换为度数
    angle_deg = math.degrees(angle_rad)
    
    return angle_deg

# (0,0),(1,1),(1,5),(3,6),(6,9),(6,13),(9,17),(16,18),(19,18),(19,19)
# (0,0),(0,20),(5,21),(8,21),(9,22),(9,28),(22,28),(29,29)
# (0,0),(8,3),(10,7),(20,8),(22,23),(22,26),(29,29)

# 示例
A = (22, 23)
B = (22, 26)
C = (29, 29)

angle = calculate_angle(A, B, C)
print(f"角度: {angle:.2f}°")
