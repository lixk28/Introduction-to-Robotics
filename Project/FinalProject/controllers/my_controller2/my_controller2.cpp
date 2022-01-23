#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdlib>
#include <ctime>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define DEBUG

using namespace std;
using namespace webots;
using namespace cv;

const double v = 20.0;
double speedForward[4] = {v, v, v, v};
double speedBackward[4] = {-v, -v, -v, -v};
double speedLeftward[4] = {0.4 * v, 0.4 * v, v, v};
double speedRightward[4] = {v, v, 0.4 * v, 0.4 * v};
double speedLeftCircle[4] = {-0.1 * v, -0.1 * v, 0.1 * v, 0.1 * v};
double speedRightCircle[4] = {0.1 * v, 0.1 * v, -0.1 * v, -0.1 * v};

void setSpeed(int keyValue, double *speed);

// param
const int mapHeight = 600;
const int mapWidth = 450;
const double worldHeight = 6;
const double worldWidth = 4.5;
const double world2pixel = mapHeight / worldHeight;
const int outlierCnt = 3; // 离群点检测范围

#define sampleNum 1000 // 采样点个数

const double dstX = -2.0;
const double dstY = -2.0;

#define INF 100000

typedef struct point
{
  int x;
  int y;

  point()
  {
  }

  point(int _x, int _y)
  {
    x = _x;
    y = _y;
  }
} point;

double distance(point p1, point p2)
{
  return abs(p1.x - p2.x) + abs(p1.y - p2.y);
  // return sqrt(pow((double)p1.x - (double)p2.x, 2.0) + pow((double)p1.y - (double)p2.y, 2.0));
}

int min(int a, int b)
{
  return a < b ? a : b;
}
int max(int a, int b)
{
  return a > b ? a : b;
}

vector<double> linspace(int a, int b, int n)
{
  vector<double> res;
  double d = (double)(max(a, b) - min(a, b)) / (double)n;
  for (int i = 0; i < n; i++)
    res.push_back((double)min(a, b) + d * i);
  return res;
}

bool is_collision(Mat map, point p1, point p2)
{
  int step_len = max(abs(p1.x - p2.x), abs(p1.y - p2.y));
  vector<double> path_x = linspace(min(p1.x, p2.x), max(p1.x, p2.x), step_len + 1);
  vector<double> path_y = linspace(min(p1.y, p2.y), max(p1.y, p2.y), step_len + 1);
  for (int i = 1; i < step_len + 1; i++)
  {
    if (map.data[(int)round(path_x[i]) * mapWidth + (int)round(path_y[i])] == 0)
    {
      return true;
    }
  }
  return false;
}

bool compare_fscore(pair<int, int> a, pair<int, int> b)
{
  return a.second < b.second;
}

vector<point> rebuild_path(vector<point> vertex2coord, map<int, int> cameFrom, int current)
{
  vector<point> path;
  path.push_back(vertex2coord[current]);

  vector<int> keys;
  for (auto it = cameFrom.begin(); it != cameFrom.end(); it++)
  {
    keys.push_back(it->first);
  }

  while (find(keys.begin(), keys.end(), current) != keys.end())
  {
    current = cameFrom[current];
    path.push_back(vertex2coord[current]);
  }
  return path;
}

vector<point> astar(int graph[][sampleNum + 2], vector<point> vertex2coord, point src, point dst)
{
  vector<int> openSet;
  vector<int> closeSet;
  map<int, int> cameFrom;
  openSet.push_back(0); // 加入起点到开集

  map<int, int> gScore;
  gScore[0] = 0; // 起点的 g 值为 0
  for (int i = 1; i < sampleNum + 2; i++)
    gScore[i] = INF;

  map<int, int> fScore;
  fScore[0] = distance(vertex2coord[0], vertex2coord[1]); // 起点的 f 值为起点和终点的距离
  for (int i = 1; i < sampleNum + 2; i++)
    fScore[0] = INF;

  while (openSet.size() > 0)
  {
    map<int, int> fScoreOpen;
    for (int &i : openSet)
    {
      fScoreOpen.insert(pair<int, int>(i, fScore[i]));
    }
    pair<int, int> min_pair = *min_element(fScoreOpen.begin(), fScoreOpen.end(), compare_fscore);
    int current = min_pair.first;

    if (current == 1) // 如果到达终点
      return rebuild_path(vertex2coord, cameFrom, current);

    remove(openSet.begin(), openSet.end(), current);
    closeSet.push_back(current);

    vector<int> neighbors;
    for (int i = 0; i < sampleNum + 2; i++) // 获取当前节点的所有邻居节点的索引
    {
      if (graph[current][i] > 0)
      {
        neighbors.push_back(i);
      }
    }

    for (int &neighbor : neighbors) // 遍历所有邻居节点
    {
      if (find(closeSet.begin(), closeSet.end(), neighbor) != closeSet.end())
        continue;

      int tentative_gscore = gScore[current] + distance(vertex2coord[current], vertex2coord[neighbor]);
      if (find(openSet.begin(), openSet.end(), neighbor) == openSet.end())
        openSet.push_back(neighbor);
      else if (tentative_gscore >= gScore[neighbor])
        continue;

      cameFrom[neighbor] = current;
      gScore[neighbor] = tentative_gscore;
      fScore[neighbor] = gScore[neighbor] + distance(vertex2coord[neighbor], vertex2coord[1]);
    }
  }
}

vector<point> prm(Mat map, point src, point dst)
{
  vector<point> vertex2coord;
  vertex2coord.push_back(src); // 索引 0 对应起点 (即小车位置)
  vertex2coord.push_back(dst); // 索引 1 对应终点

  int graph[sampleNum + 2][sampleNum + 2]; // 邻接矩阵
  memset(graph, 0, sizeof(int) * (sampleNum + 2) * (sampleNum + 2));

  while (vertex2coord.size() < sampleNum + 2)
  {
    int x = rand() % mapHeight; // 随机选取采样点
    int y = rand() % mapWidth;
    point samplePoint(x, y);
    if (map.data[x * mapWidth + y] != 0)
    {
      vertex2coord.push_back(samplePoint);
    }
  }

  for (int i = 0; i < sampleNum + 2; i++)
  {
    for (int j = 0; j < sampleNum + 2; j++)
    {
      if (i != j && distance(vertex2coord[i], vertex2coord[j]) <= 100 && is_collision(map, vertex2coord[i], vertex2coord[j]) == false)
      {
        graph[i][j] = distance(vertex2coord[i], vertex2coord[j]);
      }
    }
  }

  vector<point> path = astar(graph, vertex2coord, src, dst);
  reverse(path.begin(), path.end());

#ifdef DEBUG
  Mat tmp_map = map.clone();
  for (point &i : path)
  {
    printf("(%d, %d)\n", i.x, i.y);
    circle(tmp_map, Point(i.y, i.x), 3, 0, -1);
  }
  imshow("astar_debug", tmp_map);
  waitKey(1);
#endif

  return path;
}

double thershold = 0.06;

double convertDiffToDirection(point start, point end)
{
  
  return -atan2(end.x - start.x, end.y - start.y);;
}

char control(double currentYaw, double pathDirection)
{
  char keyValue = 'Z';
  if (abs(currentYaw - pathDirection) < thershold)
  {
    //printf("%lf\n",currentYaw-pathDirection);
    keyValue = 'W'; //forward
  }
  else
  { //find path
    double oppositeYaw = currentYaw > 0 ? currentYaw - M_PI : currentYaw + M_PI;
    printf("yaw:%lf dire:%lf opposite yaw:%lf\n", currentYaw, pathDirection,oppositeYaw);
    if (pathDirection > currentYaw && pathDirection < oppositeYaw)
    {
    keyValue = 'Q'; //turn left
    }
    else
    {
    keyValue = 'E'; //turn right
    }
  }
  return keyValue;
}

bool doweneedplan(Mat map, Mat prev_map)
{
  int cnt = 0;
  for (int i = 0; i < mapHeight; i++)
  {
    for (int j = 0; j < mapWidth; j++)
    {
      if (map.data[i * mapWidth + j] != prev_map.data[i * mapWidth + j])
        cnt++;
    }
  }
  return cnt > 10;
}

int main(int argc, char **argv)
{
  srand(time(NULL));

  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  Keyboard keyboard;
  keyboard.enable(timeStep);

  Motor *motors[4];
  char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
  double speed[4];
  for (size_t i = 0; i < 4; i++)
  {
    motors[i] = robot->getMotor(wheelsNames[i]);
    motors[i]->setPosition(std::numeric_limits<double>::infinity());
    motors[i]->setVelocity(0.0);
    speed[i] = 0;
  }

  GPS *gps = robot->getGPS("gps");
  gps->enable(timeStep);
  const double *pos; // (x, y, z)

  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  imu->enable(timeStep);
  const double *rpy; // (roll, pitch, yaw)

  Lidar *lidar = robot->getLidar("lidar");
  lidar->enable(timeStep);
  const float *lidarImage;
  int lidarRes = lidar->getHorizontalResolution(); // number of lidar points per line

  Mat prev_map(mapHeight, mapWidth, CV_8U, 255);
  Mat map(mapHeight, mapWidth, CV_8U, 255);

  // printf("%d\n", map.channels());

  int cnt = 0, maxCnt = 30;

  vector<point> path;

  while (robot->step(timeStep) != -1)
  {
    int keyValue = 0;
    keyValue = keyboard.getKey();

    pos = gps->getValues();
    double carX = pos[0];
    double carY = pos[1];
    int carPixelX = int((carX + worldWidth / 2.0) * world2pixel);
    int carPixelY = mapHeight - int((carY + worldHeight / 2.0) * world2pixel);

    int dstPixelX = int((dstX + worldWidth / 2.0) * world2pixel);
    int dstPixelY = mapHeight - int((dstY + worldHeight / 2.0) * world2pixel);

    rpy = imu->getRollPitchYaw();
    double carAngle = rpy[0]; // rpy[0] = roll, 但旋转小车时 yaw 不发生变化。
    // cout << "rpy = " << carAngle << endl;

    // step1 建图
    // Mat lidarPoints(mapHeight, mapWidth, CV_8U, 255);
    // circle(lidarPoints, Point2d(carPixelX, carPixelY), 3, 0, -1);

    if (cnt < 4) // 减速过程
      keyValue = 0;

    if (cnt == 4)
    {
      lidarImage = lidar->getRangeImage();
      double mapPointAngle, mapPointX, mapPointY;
      for (size_t i = outlierCnt; i < lidarRes - outlierCnt; ++i)
      {
        if (isfinite(lidarImage[i]) && lidarImage[i] != 0)
        {
          double outlierCheck = 0.0;
          for (int k = i - outlierCnt; k < i + outlierCnt; ++k)
          {
            outlierCheck += abs(lidarImage[i] - lidarImage[k]);
          }
          outlierCheck /= (2 * outlierCnt);
          if (outlierCheck > 0.1 * lidarImage[i])
            continue;

          mapPointAngle = carAngle + M_PI - double(i) / double(lidarRes) * 2.0 * M_PI;
          mapPointX = lidarImage[i] * cos(mapPointAngle);
          mapPointY = lidarImage[i] * sin(mapPointAngle);
          mapPointX += carX;
          mapPointY += carY;

          int imgX = int((mapPointX + worldWidth / 2.0) * world2pixel);
          int imgY = mapHeight - int((mapPointY + worldHeight / 2.0) * world2pixel);
          if (imgX >= 0 && imgX < map.cols && imgY >= 0 && imgY < map.rows)
          {
            // circle(lidarPoints, Point(imgX, imgY), 1, 0);
            circle(map, Point(imgX, imgY), 15, 0, -1);
          }
        }
      }
      // imshow("lidar points", lidarPoints);

      // imshow("map", map);
      // waitKey(1);
    }

    // step2
    // 判断是否需要规划
    // bool needPlan = doweneedplan(map, prev_map);

    // // step3
    // // 规划
    // // if (needPlan == true)
    // // {
    // // prev_map = map.clone(); // crap
    if (cnt == 10)
    {
      point car_pos(carPixelY, carPixelX);
      point dst_pos(dstPixelY, dstPixelX);
      path = prm(map, car_pos, dst_pos);
    }
    // // path = reducePath(map, path);
    // // }

    // step4
    // 控制
    if (path.size() > 1 && cnt > 10)
    {
      // printf("%d\n",path.size());
      // printf("%d %d\n",path[0].x,path[0].y);
      // printf("%d %d\n",path[1].x,path[1].y);
      int i = 1;
      if (distance(path[i], point(carPixelY, carPixelX)) < 10)
      {
        i++;
      }
      double direction = convertDiffToDirection(path[i-1], path[i]);
      keyValue = control(carAngle, direction);
    }

    cnt++;
    if (cnt >= maxCnt)
      cnt -= maxCnt;

    setSpeed(keyValue, speed);

    for (int i = 0; i < 4; ++i)
      motors[i]->setVelocity(speed[i]);
  }

  delete robot;
  return 0;
}

void setSpeed(int keyValue, double *speed)
{
  if (keyValue == 'W')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedForward[i];
  else if (keyValue == 'S')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedBackward[i];
  else if (keyValue == 'A')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedLeftward[i];
  else if (keyValue == 'D')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedRightward[i];
  else if (keyValue == 'Q')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedLeftCircle[i];
  else if (keyValue == 'E')
    for (int i = 0; i < 4; ++i)
      speed[i] = speedRightCircle[i];
  else
    for (int i = 0; i < 4; ++i)
      speed[i] = 0;
}
