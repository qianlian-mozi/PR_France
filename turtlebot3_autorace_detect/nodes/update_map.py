import cv2
import numpy as np

# 读取 map.pgm
map_path = "/home/yirenqiu/turtlebot3_ws/map_new.pgm"
map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

def read_points_from_txt(filename):
    """
    从文本文件中读取坐标点
    :param filename: 要读取的文件路径
    :return: 白色车道点和黄色车道点
    """
    white_lane_points = []
    yellow_lane_points = []

    with open(filename, 'r') as file:
        lines = file.readlines()

        # 读取白色车道点
        white_points_started = False
        yellow_points_started = False
        for line in lines:
            line = line.strip()  # 去掉每行的前后空白字符
            if line == "White lane points:":
                white_points_started = True
                continue
            elif line == "Yellow lane points:":
                yellow_points_started = True
                continue

            if white_points_started and line:
                x, y = map(float, line.split(","))
                white_lane_points.append((x, y))

            if yellow_points_started and line:
                x, y = map(float, line.split(","))
                yellow_lane_points.append((x, y))

    return white_lane_points, yellow_lane_points


# 示例：读取文件中的坐标点
filename = "/home/yirenqiu/turtlebot3_ws/points.txt"  # 修改为实际的文件路径
white_points, yellow_points = read_points_from_txt(filename)

np_white_points = np.array(white_points)
np_yellow_points = np.array(yellow_points)
# print(size(white_points))

# 修改点的值
# 假设要在 (100, 150) 的像素位置添加障碍物
# for i in range(384):
    
#     x, y = 100, i 
#     map_image[y, x] = 0  # 设置为黑色 (障碍物)
x_white = []
y_white = []
# x_white = x_white.append(np_white_points[:,0])
# y_white = y_white.append(np_white_points[:,1])
x_white = np_white_points[:,0]
y_white = np_white_points[:,1]

x_yellow = []
y_yellow= []
# x_yellow = x_yellow.append(np_yellow_points[:,0])
# y_yellow = y_yellow.append(np_yellow_points[:,1])
x_yellow = np_yellow_points[:,0]
y_yellow = np_yellow_points[:,1]

for i in range(len(x_white)):
    map_image[int(y_white[i]),int(x_white[i])] = 0
    
for i in range(len(y_yellow)):
    map_image[int(y_yellow[i]),int(x_yellow[i])] = 0
    


# 保存修改后的地图
cv2.imwrite("/home/yirenqiu/turtlebot3_ws/202412152035.pgm", map_image)

print("地图已更新并保存为 202412152035.pgm")
