import cv2
import numpy as np
import tqdm

OBSTACLE = 1
EMPTY = 0

NUM_SAMPLE = 500

SOURCE = 0
DEST = 1

# START 和 END 是矩阵中对应的坐标，而不是图像中的坐标（应该 x 和 y 对调）
START = (45, 545)
END = (750, 30)

INF = 1e8

# 计算两个像素点间的欧几里德距离
def distance(point1, point2):
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
# Dijkstra 算法
def dijkstra(graph):
  visit = [False] * (NUM_SAMPLE + 2)
  path = np.zeros(shape=(NUM_SAMPLE + 2, ), dtype=np.int32)
  dist = np.zeros(shape=(NUM_SAMPLE + 2, ), dtype=np.float32)
  for i in range(NUM_SAMPLE + 2):
    path[i] = -1
  for i in range(1, NUM_SAMPLE + 2):
    dist[i] = INF
  
  for _ in range(NUM_SAMPLE + 2):
    temp = INF
    u = -1
    for vertex in range(NUM_SAMPLE + 2):  # 找到 dist 最小的顶点 u
      if visit[vertex] == False and dist[vertex] < temp:
        temp = dist[vertex]
        u = vertex
    visit[u] = True # 标记顶点 u
    for v in range(NUM_SAMPLE + 2): # 遍历 u 的所有邻点
      if not visit[v] and graph[u][v] > 0:
        dist[v] = min(dist[v], dist[u] + graph[u][v]) # 松弛操作
        path[v] = u # 记录路径

  return dist, path

# A* 算法
def astar(graph):
  pass

def prm(roadmap):
  height, width = roadmap.shape[0], roadmap.shape[1]
  # print(height, width)
  vertex2coord = {} # 顶点编号到坐标的映射
  vertex2coord[0], vertex2coord[1] = START, END # 起点和终点
  graph = np.zeros(shape=(NUM_SAMPLE + 2, NUM_SAMPLE + 2), dtype=np.float32)  # 邻接矩阵
  i = 2
  while len(vertex2coord) < NUM_SAMPLE + 2:
    x = np.random.randint(low=0, high=height)  # 随机生成采样点 (x, y)
    y = np.random.randint(low=0, high=width)
    if roadmap[x, y] != OBSTACLE and (x, y) not in vertex2coord.keys(): # 检测是否是障碍点
      vertex2coord[i] = (x, y)
      i += 1

  for i in range(NUM_SAMPLE + 2):
    for j in range(NUM_SAMPLE + 2):
      # 检测两点之间距离是否小于 100，且连线是否会碰撞到障碍
      if i != j and distance(vertex2coord[i], vertex2coord[j]) <= 50 and collision_check(vertex2coord[i], vertex2coord[j], roadmap):
        graph[i][j] = distance(vertex2coord[i], vertex2coord[j])

  dist, path = dijkstra(graph)

  return vertex2coord, graph, dist, path

def draw_graph(graph, vertex2coord, img):
  img_graph = img
  for i in range(NUM_SAMPLE + 2):
    for j in range(NUM_SAMPLE + 2):
      if graph[i][j] > 0:
        # 注意这里坐标 x 和 y 要调换位置
        # 因为 opencv 图像的坐标和矩阵坐标是不一样的
        img_graph = cv2.line(img=img_graph, pt1=(vertex2coord[i][1], vertex2coord[i][0]), pt2=(vertex2coord[j][1], vertex2coord[j][0]), color=(0, 0, 0), thickness=1)
  cv2.imwrite("../textures/maze_graph.png", img_graph)

def draw_path(path, vertex2coord, img):
  p = []
  u = DEST
  print(path)
  while path[u] != -1:
    p.append(u)
    u = path[u]
  print(p)
  img_path = img
  for i in range(len(p) - 1):
    v1 = p[i]
    v2 = p[i + 1]
    img_path = cv2.line(img=img_path, pt1=(vertex2coord[v1][1], vertex2coord[v1][0]), pt2=(vertex2coord[v2][1], vertex2coord[v2][0]), color=(0, 0, 0), thickness=9)
  cv2.imwrite("../textures/maze_prm.png", img_path)

if __name__ == "__main__":
  img = cv2.imread("../textures/maze.png")
  # cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)
  # cv2.imshow("img", img)
  # cv2.waitKey(0)

  height, width, channels = img.shape
  print("width = {}".format(width))
  print("height = {}".format(height))
  print("channels = {}".format(channels))

  img_gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)  # 灰度化
  ret, img_threshold = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)  # 二值化
  cv2.imwrite("../textures/maze_processed.png", img_threshold)

  img_mat = np.array(img_gray)  # 转换为 numpy 数组
  img_mat = np.where(img_mat < 50, 0, 255) # 灰度值小于 50 置 0，否则置 255

  # 设置 800x600 路线图
  # 黑色像素点设置为 OBSTACLE 1
  # 白色像素点设置为 EMPTY 0
  roadmap = np.zeros(shape=(height, width), dtype=np.uint8)
  for i in range(height):
    for j in range(width):
      roadmap[i, j] = OBSTACLE if img_mat[i, j] == 0 else EMPTY

  vertex2coord, graph, dist, path = prm(roadmap)
  print(graph)
  # print(vertex2coord)
  # draw_graph(graph, vertex2coord, img)
  draw_path(path, vertex2coord, img)


