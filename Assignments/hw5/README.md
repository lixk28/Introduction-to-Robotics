编译运行方法：

```shell
cd RRT_planning
make rrt
make ctrl
```

`make rrt` 会运行 `rrt.py`。

- 生成的路径图 `maze_rrt.png` 在 `textures` 目录下。
- 其余的路径对比图（平滑前与平滑后）以及 RRT 生成树的图片在 `rrt` 目录下。

`make ctrl` 会编译小车寻线控制器，生成二进制文件。

---

完整录屏请查看 `rrt_demo.mp4`，包含整个代码运行、小车寻线的过程。

报告请查看 `report.pdf`。
