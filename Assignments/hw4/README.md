实验环境：
- Ubuntu 20.04.3 LTS x86_64
- Webots R2021a

依赖：
- opencv-python
  
依赖安装方法：
- pip：
  ```shell
  pip3 install opencv-python
  ```
- conda:
  ```shell
  conda create -n prm
  conda install opencv-python
  conda activate prm
  ```
  
运行方法：
- 首先，在项目目录下 `cd` 到 `prm` 目录，然后运行 PRM 算法（大概需要 20 - 30 秒）：
  ```shell
  cd ./PRM_planning/prm
  python3 prm.py
  ```
  程序读取 `textures` 目录下的 `maze.png` 图片，首先进行处理，生成 `maze_processed.png` 图片（同样在 `textures` 目录下），然后根据 PRM 算法画出规划好的路径，生成 `maze_prm.png` 图片。

- 然后，`cd` 到 `controllers/LineFollowing` 目录，`make`：
  ```shell
  cd ../controllers/LineFollowing
  make
  ```

  会生成可执行的二进制文件，可以直接在 webots 中模拟了。
- 最后，用 webots 打开 `worlds` 目录下的 `prm_world.wbt` 世界，在节点列表中的 `RectangleArea` 节点，将其地板材质的 url 修改为 `../textures/maze_prm.png`，再直接进行模拟即可。
