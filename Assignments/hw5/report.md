# HW4：RRT 算法路径规划

|                                |                     |
| :----------------------------- | :------------------ |
| 学号：19335109                 | 课程：机器人导论    |
| 姓名：李雪堃                   | 学期：Fall 2021     |
| 专业：计算机科学与技术（超算） | 教师：成慧          |
| 邮箱：i@xkun.me                | TAs：黄家熙、李皖越 |

---

***Table of Contents***

[toc]

---

## (一) 实验要求

- 绿色方块代表起始位置，红色方块代表目标位置，要求在已知地图全局信息的情况下，使用 RRT 算法规划一条尽可能短的轨迹，控制机器人从绿色走到红色。

- 给定了迷宫 webots 模型，地图的全局信息通过读取 `maze.png` 这个图片来获取。

## (二) 实验环境

- Ubuntu 20.04.3 LTS x86_64
- Webots R2021a
- opencv-python 4.5.2.52
- numpy 1.20.2

## (三) 实验过程和核心代码

### (1) RRT 算法



### (2) 路径平滑

#### (i) 三角形平滑



#### (ii) 梯度下降平滑



### (3) 小车寻线





## (四) 实验结果

录制的视频在 `rrt_demo.mp4`。

截屏



## (五) 遇到的问题与总结

RRT 中间画蛇添足

路径平滑问题



## (六) 参考资料

- https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
- http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
- https://vslam.net/2021/03/28/route_planning/%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%EF%BC%88%E5%85%AB%EF%BC%89-RRT%E7%AE%97%E6%B3%95/
- https://dlonng.com/posts/rrt
- https://www.geeksforgeeks.org/check-whether-triangle-is-valid-or-not-if-three-points-are-given/
- https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
- https://blog.csdn.net/qq_29796781/article/details/80113026
