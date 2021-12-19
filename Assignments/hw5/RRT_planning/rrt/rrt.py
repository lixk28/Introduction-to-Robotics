import cv2 as cv
import numpy as np
import math
import argparse
from tqdm import tqdm

OBSTACLE = 1  # 障碍点
EMPTY = 0 # 空白点

NUM_SAMPLE = 100000 # 采样点个数
NUM_VERTEX = NUM_SAMPLE + 2 # 顶点总数

# SOURCE 和 DEST 是起点和终点的索引，NIL 代表空节点
SOURCE = 0
DEST = 1
NIL = -1

# 起点和终点在矩阵中对应的坐标，而不是图像中的坐标（应该 x 和 y 对调）
SOURCE_COORD = (45, 545)
DEST_COORD = (750, 30)

INF = 1e8

EXPLORE_RATE = 0.9  # 探索率

STEP_SIZE = 10 # 步长

SMOOTH_ITER = 5000
ALPHA = 0.4 # 保守率
BETA = 0.6  # 平滑率

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

# 获得像素坐标到该像素是否是障碍的 0-1 矩阵
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
  print("map height: {}".format(height))
  print("map width: {}".format(width))
  
  vertex2coord = {} # 每个节点索引到它矩阵坐标的映射
  vertex2coord[SOURCE] = SOURCE_COORD

  parent = {} # 每个节点索引到它父节点索引的映射
  parent[SOURCE] = NIL  # 起点和终点的父节点设置为NIL
  parent[DEST] = NIL

  i = 2
  iterator = tqdm(iterable=range(NUM_SAMPLE), desc="RRT is in progress", bar_format='{l_bar}{bar:10}{r_bar}{bar:-10b}')
  for _ in iterator:
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

    if nearest_dist <= STEP_SIZE: # 如果采样点到最近点的距离小于步长，采样点就作为新生成的点
      x_new, y_new = x_rand, y_rand
    else: # 否则向采样点推进一个步长
      theta = math.atan2(y_rand - y_near, x_rand - x_near)  # 计算角度
      x_new = int(np.ceil(x_near + STEP_SIZE * np.cos(theta))) # 根据步长计算新坐标
      y_new = int(np.ceil(y_near + STEP_SIZE * np.sin(theta)))

    if not collision_check((x_near, y_near), (x_new, y_new), map):  # 如果新生成的点不会碰撞
      vertex2coord[i] = (x_new, y_new)
      parent[i] = nearest_neighbor
      i += 1
      # print("new vertex is ({}, {}), its nearest neighbor is ({}, {})".format(x_new, y_new, x_near, y_near))

    if euclid_distance((x_new, y_new), DEST_COORD) < STEP_SIZE: # 如果新节点到目标节点的距离小于步长
      parent[DEST] = i - 1  # 设置 DEST 的父节点是新节点，并跳出循环
      iterator.close()
      print("Reach the destination and stop searching")
      print("The total number of rrt nodes is {}".format(len(vertex2coord)+1), end='\n\n')
      break
      
  vertex2coord[DEST] = DEST_COORD # 最后将终点加入

  return vertex2coord, parent

# 检查三个点是否会构成三角形
def triangle_check(pt1, pt2, pt3):
  # 路径为 pt1 -> pt2 -> pt3
  area = pt1[0] * (pt2[1] - pt3[1]) + \
         pt2[0] * (pt3[1] - pt1[1]) + \
         pt3[0] * (pt1[1] - pt2[1])
  return False if area == 0 else True

# 三角形平滑
def triangle_smoothing(path, map):
  # 一拍脑子想出的三角形平滑算法
  path_smooth = []
  path_smooth.append(SOURCE_COORD)
  it = tqdm(iterable=range(1, len(path)-1, 2), desc="Triangle smoothing", \
            bar_format='{l_bar}{bar:10}{r_bar}{bar:-10b}')
  for i in it:
    pt_prev, pt_curr, pt_next = path[i-1], path[i], path[i+1]
    if triangle_check(pt_prev, pt_curr, pt_next) and not collision_check(pt_prev, pt_next, map):  # 如果构成三角形，并且不会碰撞
      path_smooth.append(pt_next) # 舍弃掉中间点 pt_curr
    else:
      path_smooth.append(pt_curr)
      path_smooth.append(pt_next)
  return path_smooth

# 梯度下降平滑
def gradient_smoothing(path, map):
  path_smooth = path.copy()
  path_smooth = np.asarray(path_smooth).astype(np.int32)
  it = tqdm(iterable=range(SMOOTH_ITER), desc="Gradient smoothing", \
            bar_format='{l_bar}{bar:10}{r_bar}{bar:-10b}')
  for _ in it:
    for i in range(1, len(path)-1, 1):
      path_smooth[i][0] = int(np.ceil(path_smooth[i][0] + \
                                      ALPHA * (path[i][0] - path_smooth[i][0]) + \
                                      BETA * (path_smooth[i-1][0] - 2 * path_smooth[i][0] + path_smooth[i+1][0])))
      path_smooth[i][1] = int(np.ceil(path_smooth[i][1] + \
                                      ALPHA * (path[i][1] - path_smooth[i][1]) + \
                                      BETA * (path_smooth[i-1][1] - 2 * path_smooth[i][1] + path_smooth[i+1][1])))
  return path_smooth

# 画出 RRT 生成树和初始路径
def draw_tree(vertex2coord, parent, path):
  img = cv.imread("../textures/maze.png")
  for i in range(len(path)-1):  # 画初始路径
    cv.line(img=img, pt1=(path[i][1], path[i][0]), pt2=(path[i+1][1], path[i+1][0]), \
            color=(255, 0, 255), thickness=3, lineType=cv.LINE_AA)
  flag = [False for _ in range(len(vertex2coord))]  # flag 标记每个节点，是否已经画过
  for i in range(len(vertex2coord)):  # 回溯每个节点的所有祖先节点
    while i != NIL:
      if flag[i] == False:  # 已画过的不需要再画
        cv.circle(img=img, center=(vertex2coord[i][1], vertex2coord[i][0]), radius=3, \
                  color=(255, 0, 0), thickness=1, lineType=cv.LINE_AA)
        flag[i] = True
      i = parent[i]
  cv.imwrite("./rrt_tree.png", img)

# 画出给定路径
def draw_path(path, img_name, draw_point=False):
  img = cv.imread("../textures/maze.png")
  if draw_point == True:
    for i in range(len(path)):
      cv.circle(img=img, center=(path[i][1], path[i][0]), radius=5, \
                color=(255, 0, 0), thickness=1, lineType=cv.LINE_AA)
  for i in range(len(path)-1):
    cv.line(img=img, pt1=(path[i][1], path[i][0]), pt2=(path[i+1][1], path[i+1][0]), \
            color=(0, 0, 0), thickness=3, lineType=cv.LINE_AA)
  cv.imwrite(img_name, img)

def draw_rrt(vertex2coord, parent, map):
  path = []
  i = DEST
  while i != NIL:
    path.append(vertex2coord[i])
    i = parent[i]
  path.reverse()
  # print(len(path))
  # print(path)

  print("Path smoothing:")
  path_triangle_smooth = triangle_smoothing(path, map)
  path_gradient_smooth = gradient_smoothing(path_triangle_smooth, map)
  print("The length of smoothed path is {}".format(len(path_gradient_smooth)))

  # 原始路径、三角形平滑路径、梯度下降平滑路径的对比
  draw_path(path=path, img_name="./raw_path.png", draw_point=True)
  draw_path(path=path_triangle_smooth, img_name="./triangle_smooth_path.png", draw_point=True)
  draw_path(path=path_gradient_smooth, img_name="./gradient_smooth_path.png", draw_point=True)

  # 画出 RRT 生成树
  draw_tree(vertex2coord, parent, path)

  # 画出最终的路径，用于小车寻线
  draw_path(path_gradient_smooth, "../textures/maze_rrt.png")

if __name__ == "__main__":
  img = img_process() # 获得处理后的图像和像素矩阵
  # print(type(img))

  map = get_map(img)

  vertex2coord, parent = rrt(map)
  # print(vertex2coord)
  # print(parent)

  draw_rrt(vertex2coord, parent, map)

