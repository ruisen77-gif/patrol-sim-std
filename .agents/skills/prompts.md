# 提示词合集

七个提示词，按角色取用。所有提示词都遵守同一口径：20×20 网格，坐标 0–19，原点左上，x 右 y 下。

## 1. 嵌入式仿真（backend/sim/robot_sim.py）

```
写 backend/sim/robot_sim.py，纯 Python 标准库，20*20 网格，坐标 0-19，y 向下递增。
提供模块级状态和函数：
1. move(direction)，direction 取 up/down/left/right；
   先判断下一格是否越界或位于障碍物，若是则返回 {"ok":False,"msg":"具体原因"} 且坐标不变；
   成功则更新坐标并同步 angle（0右 90下 180左 270上），返回 {"ok":True}。
2. get_state() 返回 {"x":int,"y":int,"angle":int,"status":"idle"或"moving"或"finished"}。
3. follow(points, speed)，points 为 [[x,y],...]，用后台线程逐点行进，
   每格间隔按 speed 换算（0.5x 慢、4x 快），行进中 status 为 moving，结束时为 finished。
4. stop() 终止线程并把 status 置回 idle。
5. set_map(obstacles, size) 供后端注入地图。
模块底部写 if __name__ == "__main__" 自测：连续调用 move、测试撞障碍、测试越界、测试 follow。
输出完整全部代码。
```

## 2. 算法（backend/sim/path_planner.py）

```
写 backend/sim/path_planner.py，纯 Python 标准库。
1. bfs(size, obstacles, start, end)：四方向 BFS，不得穿过障碍物。
   返回 {"ok":True,"path":[[x,y],...],"msg":""} 或 {"ok":False,"path":[],"msg":"无法找到路径"}。
   起点等于终点时返回只含一个点的路径。
2. patrol(size, obstacles, points)：按顺序对相邻两点调用 bfs，
   把各段路径去掉首点后依次拼接；某段失败则跳过该巡检点并记录到 unreachable_points。
底部写自测：空地寻路、终点被障碍包围、起点终点重合、三个巡检点串联，打印结果。
输出完整全部代码。
```

## 3. 人工智能虚拟传感器（backend/sim/sensor_sim.py）

```
写 backend/sim/sensor_sim.py，纯 Python 标准库。
1. detect(state, obstacles, size)：state 含 x、y、angle。
   按 angle 换算出前、左、右三个相邻格坐标，检查是否越界或位于障碍物，
   返回 {"front_blocked":bool,"left_blocked":bool,"right_blocked":bool,
         "collision":bool,"out_of_bound":bool}
   collision 指当前坐标恰好落在障碍物上；out_of_bound 指坐标超出 0-19 范围。
2. 若输入坐标非法，返回各项 False 并带 msg 说明，不要抛异常。
底部写自测：空旷处、正前方有障碍、三面被围、边界角落、越界坐标，打印结果。
输出完整全部代码。
```

## 4. Flask 后端（backend/app.py）

```
写 backend/app.py，Flask 应用，目录为 backend/，同目录下有 sim/ 包。
导入 from sim import robot_sim, path_planner, sensor_sim。
启动时用 set_map 注入 20*20 地图，障碍物写死一组固定坐标（约 12 个点，留出可通行通道）。
实现接口，全部允许跨域：
GET  /api/map     返回 {"size":20,"obstacles":[...],"start":[x,y]}
GET  /api/robot   返回 robot_sim.get_state()
POST /api/move    读 {"direction":...} 调 robot_sim.move，失败返回 409 和 msg
POST /api/patrol  读 {"points":[...],"speed":1}，调 path_planner.patrol，
                  失败返回 409 和 {"ok":false,"msg":"目标点不可达"}，
                  成功则调用 robot_sim.follow(plan, speed)，返回 {"ok":true,"plan":[...]}
POST /api/stop    调 robot_sim.stop()
GET  /api/sensor  合并 sensor_sim.detect 结果与巡检进度（已完成点数、总点数、剩余步数）
用 Flask 的 static_folder 指向前端 ../frontend，访问 / 返回 index.html。
底部写启动代码，监听 0.0.0.0:5000（本机 5000 被 AirPlay 占用时用 PORT 改 5050）。
输出完整全部代码。
```

## 5. 科技风前端（frontend/index.html）

```
写 frontend/index.html 单文件，深色科技控制台风格，CSS 与 JS 全部内联，不用外部 CDN。
配色：底 #0a1929，主色 #1e90ff，辅助 #00e5ff，告警黄 #ffcc00，危险红 #ff4757，文字 #d6e4ff；
面板半透明、边框发光、四角 HUD 装饰线、右上角状态灯。

布局三区：
顶部：标题「智能巡检机器人二维仿真系统」、副标题「BFS路径规划｜虚拟传感器｜自动巡检｜仿真运动控制」。
左侧：上下左右四个方向按钮；启动自动巡检、停止任务两个按钮；
      速度滑块 0.5x/1x/2x/4x；提示文字「点击地图可添加巡检点」。
中间：20×20 Canvas 网格；障碍物深色块加红边；机器人蓝色发光块并显示朝向；
      待走路径青色连线加圆点，已走过路径浅蓝；巡检点标 P1/P2/P3，当前目标黄色高亮。
底部：五张状态卡，分别显示机器人坐标、运动状态、传感器状态、告警信息、巡检进度。

行为：
1. 每 1 秒轮询 GET /api/robot 与 GET /api/sensor 刷新画面；
2. 方向键请求 POST /api/move，返回 409 时弹出告警弹窗显示 msg；
3. 点击网格把坐标加入巡检点列表并在地图标注；
4. 启动巡检把点位与速度 POST /api/patrol，失败弹窗提示；
5. 停止按钮请求 POST /api/stop；
6. 碰撞、越界、不可达三类情况弹出半透明告警框，可点击关闭。
不要实现日志面板。输出完整 HTML。
```

## 6. 容器与 CI

```
项目根目录为 patrol-sim，backend/app.py 是 Flask 入口，frontend/index.html 是页面，
backend/requirements.txt 已列依赖。
1. 写 Dockerfile：基于 python:3.11-slim，复制 backend 与 frontend，
   安装 requirements，工作目录 backend，启动 python app.py，暴露端口（默认 5050）。
2. 写 docker-compose.yml：单服务 web，构建上下文为根目录，端口映射到容器端口。
3. 写 .github/workflows/ci.yml：在 push 与 PR 时触发，
   安装 Python 与依赖，运行 python -m compileall backend 与 pytest backend/tests。
4. 写 backend/tests/test_sim.py 骨架：至少覆盖移动撞墙、BFS 不可达两类用例。
输出四份完整文件。
```

## 7. 文档与协作约定

```
把 AGENTS.md 的协作约定写成可直接使用的内容，包含：
项目目标与当前范围（基础版+标准版+七项拓展）、目录职责（frontend/backend/sim/docs 各放什么）、
开发约定（坐标原点左上、x右y下、20*20、机器人字段、接口返回格式）、
常用命令（后端启动、前端打开、docker compose up、pytest）、
协作边界（五个人各自维护哪些文件，禁止跨模块改代码，改接口须全员同意）。
输出可直接粘贴的 Markdown。
```
