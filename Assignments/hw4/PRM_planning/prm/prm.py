import cv2
import numpy as np

OBSTACLE = 1
EMPTY = 0

NUM_SAMPLE = 1000

SOURCE = 0
DEST = 1

# START 和 END 是矩阵中对应的坐标，而不是图像中的坐标（应该 x 和 y 对调）
START = (45, 545)
END = (750, 30)

INF = 1e8

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

# # Dijkstra 算法
# def dijkstra(graph):
#   visit = [False] * (NUM_SAMPLE + 2)
#   path = np.zeros(shape=(NUM_SAMPLE + 2, ), dtype=np.int32)
#   dist = np.zeros(shape=(NUM_SAMPLE + 2, ), dtype=np.float32)
#   for i in range(NUM_SAMPLE + 2):
#     path[i] = -1
#   for i in range(1, NUM_SAMPLE + 2):
#     dist[i] = INF
  
#   for _ in range(NUM_SAMPLE + 2):
#     temp = INF
#     u = -1
#     for vertex in range(NUM_SAMPLE + 2):  # 找到 dist 最小的顶点 u
#       if visit[vertex] == False and dist[vertex] < temp:
#         temp = dist[vertex]
#         u = vertex
#     visit[u] = True # 标记顶点 u
#     for v in range(NUM_SAMPLE + 2): # 遍历 u 的所有邻点
#       if not visit[v] and graph[u][v] > 0:
#         dist[v] = min(dist[v], dist[u] + graph[u][v]) # 松弛操作
#         path[v] = u # 记录路径

#   return dist, path

# 回溯路径
def reconstruct_path(came_from, current):
  path = []
  path.append(current)
  while current in came_from.keys():
    current = came_from[current]
    path.append(current)
  return path

# A* 算法
def a_star(graph, vertex2coord):
  open_set = []
  open_set.append(SOURCE)  # 加入起点
  close_set = []
  came_from = {}
  
  g_score = {}
  g_score[SOURCE] = 0  # 起点的 g 值为 0
  for i in range(1, NUM_SAMPLE + 2):
    g_score[i] = INF  # 其余点为 INF

  f_score = {}
  f_score[SOURCE] = manhatten_distance(vertex2coord[0], vertex2coord[1])
  for i in range(1, NUM_SAMPLE + 2):
    f_score[i] = INF

  while len(open_set) > 0:
    f_score_open = {}
    for i in open_set:
      f_score_open[i] = f_score[i]
    current = min(f_score_open, key=lambda x : f_score_open[x]) # 在开集中选取 f 值最小的节点作为当前节点

    if current == DEST:  # 如果到达终点
      return reconstruct_path(came_from, current)

    open_set.remove(current)
    close_set.append(current)

    neighbors = []
    for i in range(NUM_SAMPLE + 2): # 获取 current 节点所有的邻居
      if graph[current][i] > 0:
        neighbors.append(i)

    for neighbor in neighbors:
      if neighbor in close_set:
        continue

      tentative_gscore = g_score[current] + manhatten_distance(vertex2coord[current], vertex2coord[neighbor])
      if neighbor not in open_set:
        open_set.append(neighbor)
      elif tentative_gscore >= g_score[neighbor]:
        continue

      came_from[neighbor] = current
      g_score[neighbor] = tentative_gscore
      f_score[neighbor] = g_score[neighbor] + manhatten_distance(vertex2coord[neighbor], vertex2coord[DEST])

  # return False

def prm(roadmap):
  height, width = roadmap.shape[0], roadmap.shape[1]
  # print(height, width)
  vertex2coord = {} # 采样点的编号到图像坐标的映射
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
      if i != j and manhatten_distance(vertex2coord[i], vertex2coord[j]) <= 100 and collision_check(vertex2coord[i], vertex2coord[j], roadmap):
        graph[i][j] = manhatten_distance(vertex2coord[i], vertex2coord[j])

  path = a_star(graph, vertex2coord)

  return graph, vertex2coord, path

def draw_graph(graph, vertex2coord, img):
  img_graph = img
  for i in range(NUM_SAMPLE + 2):
    img_graph = cv2.circle(img=img_graph, center=(vertex2coord[i][1], vertex2coord[i][0]), radius=3, color=(255, 0, 0))
  cv2.imwrite("../textures/maze_graph.png", img_graph)

def draw_path(path, vertex2coord, img):
  # print(path)
  path.reverse()
  img_path = img
  for i in range(len(path) - 1):
    v1 = path[i]
    v2 = path[i + 1]
    # 注意这里坐标 x 和 y 要调换位置
    # 因为 opencv 图像的坐标和矩阵坐标是不一样的
    img_path = cv2.line(img=img_path, pt1=(vertex2coord[v1][1], vertex2coord[v1][0]), pt2=(vertex2coord[v2][1], vertex2coord[v2][0]), color=(0, 0, 0), thickness=3)
  cv2.imwrite("../textures/maze_prm.png", img_path)

if __name__ == "__main__":
  img = cv2.imread("../textures/maze.png", cv2.IMREAD_UNCHANGED)
  # cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)
  # cv2.imshow("img", img)
  # cv2.waitKey(0)

  # height, width, channels = img.shape
  # print("width = {}".format(width))
  # print("height = {}".format(height))
  # print("channels = {}".format(channels))

  img_gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)  # 灰度化
  height, width = img_gray.shape
  for i in range(height):
    for j in range(width):
      img_gray[i, j] = 0 if img_gray[i, j] < 50 else 255
  # ret, img_threshold = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)  # 二值化
  
  black_pixel_list = []
  for i in range(height):
    for j in range(width):
      if img_gray[i][j] == 0:
        black_pixel_list.append((j, i)) # holy shit
  for b in black_pixel_list:
    img_gray = cv2.circle(img=img_gray, center=b, radius=20, color=0, thickness=-1)

  cv2.imwrite("../textures/maze_processed.png", img_gray)

  img_mat = np.array(img_gray)  # 转换为 numpy 数组
  img_mat = np.where(img_mat < 50, 0, 255) # 灰度值小于 50 置 0，否则置 255

  # 设置 800x600 路线图
  # 黑色像素点设置为 OBSTACLE 1
  # 白色像素点设置为 EMPTY 0
  roadmap = np.zeros(shape=(height, width), dtype=np.uint8)
  for i in range(height):
    for j in range(width):
      roadmap[i, j] = OBSTACLE if img_mat[i, j] == 0 else EMPTY

  graph, vertex2coord, path = prm(roadmap)
  print(graph)
  # print(vertex2coord)
  # draw_graph(graph, vertex2coord, img)
  draw_path(path, vertex2coord, img)


