import cv2 as cv
import numpy as np
import math
import argparse
from tqdm import tqdm
from scipy.interpolate import splrep, splev

OBSTACLE = 1  # 障碍点
EMPTY = 0 # 空白点

NUM_SAMPLE = 100000 # 采样点个数
NUM_VERTEX = NUM_SAMPLE + 2 # 顶点总数

# SOURCE 和 DEST 是起点和终点的索引
SOURCE = 0
DEST = 1
NIL = -1

# 起点和终点在矩阵中对应的坐标，而不是图像中的坐标（应该 x 和 y 对调）
SOURCE_COORD = (45, 545)
DEST_COORD = (750, 30)

INF = 1e8

EXPLORE_RATE = 0.9  # 探索率

STEP_SIZE = 20 # 步长

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

  cv.imwrite("../textures/maze_processed.png", img) # 保存处理后的图像
  return img

def get_map(img):
  height, width = img.shape
  map = np.zeros(shape=(height, width), dtype=np.int8)  # 代表像素是否是障碍的矩阵
  for i in range(height):
    for j in range(width):
      map[i, j] = OBSTACLE if img[i, j] == 0 else EMPTY
  return map

# 计算两个像素点的曼哈顿距离
def manhatten_distance(point1, point2):
  return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

# 计算两个像素点间的欧几里德距离
def euclid_distance(point1, point2):
  return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

# 在已构造的生长树中找到距离采样点最近的节点
def find_nearest_neighbor(vertex2coord, point):
  nearest_neighbor = NIL
  nearest_dist = INF
  for vertex, coord in vertex2coord.items():
    if euclid_distance(coord, point) < nearest_dist:
      nearest_dist = euclid_distance(coord, point)
      nearest_neighbor = vertex
  return nearest_neighbor, nearest_dist

# 检测两个像素点之间的连线是否经过障碍，以及 point2 是否会超出地图边界
def collision_check(point1, point2, map):
  height, width = map.shape
  if point2[0] < 0 or point2[0] > height or point2[1] < 0 or point2[1] > width: # 边界检测
    return True
  step_len = max(abs(point1[0] - point2[0]), abs(point1[1] - point2[1]))
  path_x = np.linspace(point1[0], point2[0], step_len + 1)
  path_y = np.linspace(point1[1], point2[1], step_len + 1)
  for i in range(1, step_len + 1):
    if map[int(np.ceil(path_x[i])), int(np.ceil(path_y[i]))] == OBSTACLE:
      return True # 有碰撞
  return False  # 无碰撞

# RRT 算法 (goal-based)
def rrt(map):
  height, width = map.shape
  print(height, width)
  
  vertex2coord = {} # 每个节点索引到它矩阵坐标的映射
  vertex2coord[SOURCE] = SOURCE_COORD

  parent = {} # 每个节点索引到它父节点索引的映射
  parent[SOURCE] = NIL  # 起点和终点的父节点设置为NIL
  parent[DEST] = NIL

  i = 2
  for _ in tqdm(range(NUM_SAMPLE + 2)):
    # print(len(vertex2coord))
    # print("i = {}".format(i))
    if np.random.binomial(1, EXPLORE_RATE) == 1:  # 选择探索
      x_rand = np.random.randint(low=0, high=height) # 随机生成采样点 (x_rand, y_rand)，注意这是在矩阵中的坐标
      y_rand = np.random.randint(low=0, high=width)
    else: # 选择向终点前进，以终点为采样点
      x_rand, y_rand = DEST_COORD

    # fxxk, this check is needless !
    # if map[x_rand, y_rand] != OBSTACLE: # 若采样点是障碍点，则放弃该点，继续采样
    #   continue

    nearest_neighbor, nearest_dist = find_nearest_neighbor(vertex2coord, (x_rand, y_rand))  # 找到生长树中离采样点最近的节点
    x_near, y_near = vertex2coord[nearest_neighbor]

    if nearest_dist <= STEP_SIZE: # 如果采样点到最近点的距离小于步长
      x_new, y_new = x_rand, y_rand
    else: # 否则向采样点推进一个步长
      theta = math.atan2(y_rand - y_near, x_rand - x_near)  # 计算角度
      x_new = int(np.ceil(x_near + STEP_SIZE * np.cos(theta))) # 根据步长计算新坐标
      y_new = int(np.ceil(y_near + STEP_SIZE * np.sin(theta)))

    if not collision_check((x_near, y_near), (x_new, y_new), map):  # 不会碰撞
      vertex2coord[i] = (x_new, y_new)
      parent[i] = nearest_neighbor
      i += 1
      # print("new vertex is ({}, {}), its nearest neighbor is ({}, {})".format(x_new, y_new, x_near, y_near))

    if euclid_distance((x_new, y_new), DEST_COORD) < STEP_SIZE: # 如果新节点到目标节点的距离小于步长
      parent[DEST] = i - 1  # 设置 DEST 的父节点是新节点，并跳出循环
      print("reach the destination, break")
      break
      
  vertex2coord[DEST] = DEST_COORD

  return vertex2coord, parent

def draw_tree(vertex2coord, parent):
  # todo
  pass

def draw_path(vertex2coord, parent):
  img = cv.imread("../textures/maze.png") # 以 BGR 格式读取图片
  i = DEST
  while parent[i] != NIL:
    v1 = i
    v2 = parent[i]
    cv.line(img=img, pt1=(vertex2coord[v1][1], vertex2coord[v1][0]), pt2=(vertex2coord[v2][1], vertex2coord[v2][0]), \
            color=(0, 0, 0), thickness=3, lineType=cv.LINE_AA)
    i = parent[i]
  cv.imwrite("../textures/maze_rrt.png", img)

if __name__ == "__main__":
  img = img_process() # 获得处理后的图像和像素矩阵
  # print(type(img))

  map = get_map(img)

  vertex2coord, parent = rrt(map)
  # print(vertex2coord)
  # print(parent)

  draw_path(vertex2coord, parent)

