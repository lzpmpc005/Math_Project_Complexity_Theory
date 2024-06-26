import numpy as np
import matplotlib.pyplot as plt

# 示例距离矩阵
distance_matrix = np.array([
        [0.0, 16.5, 15.4, 13.5, 15.5, 38.0, 9.4, 29.6, 20.8, 28.4, 11.7, 8.2, 25.5, 4.6, 22.1],
        [16.5, 0.0, 15.4, 32.7, 9.6, 13.8, 35.2, 26.2, 26.0, 57.2, 11.5, 23.8, 65.8, 12.6, 34.7],
        [15.4, 15.4, 0.0, 42.8, 9.8, 26.5, 33.7, 14.8, 10.0, 46.2, 21.6, 20.0, 66.6, 16.0, 45.4],
        [13.5, 32.7, 42.8, 0.0, 37.0, 25.5, 6.8, 53.6, 53.4, 15.8, 25.4, 10.3, 28.1, 13.8, 8.9],
        [15.5, 9.6, 9.8, 37.0, 0.0, 20.6, 39.6, 20.6, 20.2, 40.4, 15.8, 28.1, 60.8, 16.9, 39.4],
        [38.0, 13.8, 26.5, 25.5, 20.6, 0.0, 28.0, 56.4, 67.0, 36.8, 8.9, 19.6, 52.1, 14.8, 28.3],
        [9.4, 35.2, 33.7, 6.8, 39.6, 28.0, 0.0, 45.4, 31.0, 23.5, 11.4, 4.0, 21.2, 7.6, 15.2],
        [29.6, 26.2, 14.8, 53.6, 20.6, 56.4, 45.4, 0.0, 13.4, 88.6, 32.5, 32.8, 101.5, 29.0, 56.0],
        [20.8, 26.0, 10.0, 53.4, 20.2, 67.0, 31.0, 13.4, 0.0, 99.2, 32.2, 29.3, 98.8, 25.0, 55.8],
        [28.4, 57.2, 46.2, 15.8, 40.4, 36.8, 23.5, 88.6, 99.2, 0.0, 28.8, 22.2, 30.9, 26.4, 7.3],
        [11.7, 11.5, 21.6, 25.4, 15.8, 8.9, 11.4, 32.5, 32.2, 28.8, 0.0, 8.0, 49.2, 7.3, 28.2],
        [8.2, 23.8, 20.0, 10.3, 28.1, 19.6, 4.0, 32.8, 29.3, 22.2, 8.0, 0.0, 33.8, 4.7, 18.1],
        [25.5, 65.8, 66.6, 28.1, 60.8, 52.1, 21.2, 101.5, 98.8, 30.9, 49.2, 33.8, 0.0, 27.6, 31.3],
        [4.6, 12.6, 16.0, 13.8, 16.9, 14.8, 7.6, 29.0, 25.0, 26.4, 7.3, 4.7, 27.6, 0.0, 14.0],
        [22.1, 34.7, 45.4, 8.9, 39.4, 28.3, 15.2, 56.0, 55.8, 7.3, 28.2, 18.1, 31.3, 14.0, 0.0]
    ])

# 将距离矩阵转换为点集
num_points = len(distance_matrix)
points = np.zeros((num_points, 2))
for i in range(num_points):
    points[i, 0] = i  # 点的索引
    points[i, 1] = distance_matrix[i, (i + 1) % num_points]  # 按顺序获取距离

# 画平行坐标图
plt.figure(figsize=(8, 6))
for i in range(num_points):
    plt.plot([0, 1], [i, (i + 1) % num_points], color='b', linewidth=2)  # 平行坐标线
plt.scatter(points[:, 0], points[:, 1], color='r', marker='o', s=100)  # 点的散点图
for i, (x, y) in enumerate(points):
    plt.text(x, y, str(i), fontsize=12, ha='right', va='bottom')  # 标注点的索引
plt.xlabel('Index')
plt.ylabel('Distance')
plt.title('Parallel Coordinates Plot of Distance Matrix')
plt.grid(True)
plt.show()
