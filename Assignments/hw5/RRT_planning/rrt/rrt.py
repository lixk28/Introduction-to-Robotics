import cv2 as cv
import numpy as np
import tqdm

OBSTACLE = 1
EMPTY = 0

NUM_SAMPLE = 1000

# SOURCE 和 DEST 是起点和终点的索引
SOURCE = 0
DEST = 1

# START 和 END 是起点和终点在矩阵中对应的坐标，而不是图像中的坐标（应该 x 和 y 对调）
START = (45, 545)
END = (750, 30)

INF = 1e8

# 图像处理
def img_process():
  img = cv.imread("../textures/maze.png") # 以 BGR 格式读取图片

  img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # 灰度化
  height, width = img.shape
  for i in range(height):
    for j in range(width):
      img[i, j] = 0 if img[i, j] < 50 else 255  # 二值化

  black_pixel_coord_list = []
  for i in range(height):
    for j in range(width):
      if img[i, j] == 0:
        black_pixel_coord_list.append((j, i)) # 黑色像素在图像中的坐标为 (j, i)
  for b in black_pixel_coord_list:  # 膨胀处理
    cv.circle(img=img, center=b, radius=20, color=0, thickness=-1)

  map = np.zeros(shape=(height, width), dtype=np.int8)  # 代表像素是否是障碍的矩阵
  for i in range(height):
    for j in range(width):
      map[i, j] = OBSTACLE if img[i, j] == 0 else EMPTY

  cv.imwrite("../textures/maze_processed.png", img) # 保存处理后的图像

  return img, map

# 计算两个像素点的曼哈顿距离
def manhatten_distance(point1, point2):
  return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

# 计算两个像素点间的欧几里德距离
def euclid_distance(point1, point2):
  return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

# 检测两个像素点之间的连线是否经过障碍
def collision_check(point1, point2, roadmap):
  step_len = max(abs(point1[0] - point2[0]), abs(point1[1] - point2[1]))
  path_x = np.linspace(point1[0], point2[0], step_len + 1)
  path_y = np.linspace(point1[1], point2[1], step_len + 1)
  for i in range(1, step_len + 1):
    if roadmap[int(np.ceil(path_x[i])), int(np.ceil(path_y[i]))] == OBSTACLE:
      return False
  return True

def rrt():
  pass

if __name__ == "__main__":
  img, map = img_process() # 获得处理后的图像和像素矩阵
  # print(type(img))

  
  

  pass

