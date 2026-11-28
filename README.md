> **在线演示：** 本地运行，见下方「快速开始」（未部署公有云，不虚构演示地址）

# 智能巡检机器人二维仿真系统

在浏览器里操作的巡检场景：20×20 二维地图，可手动开动机器人，也可点选巡检点让它按 BFS 规划出的最短路径自动巡检，虚拟传感器持续探测并弹出碰撞、越界告警。

纯软件仿真，不需要实体机器人、GPU 或付费大模型 API。

## 技术选型

| 项 | 选型 |
|---|---|
| 前端 | 原生 HTML / CSS / JavaScript，单文件，无构建步骤 |
| 后端 | Python Flask，九个 REST 接口 |
| 绘图 | Canvas 2D，DPR 适配 |
| 仿真 | 纯 Python 标准库：机器人运动、BFS 寻路、虚拟传感器 |
| CI流水线 | GitHub Actions |
| Docker部署 | Docker Compose |

## 快速开始

```bash
# Linux / MacOS
python3 -m venv .venv
source .venv/bin/activate
pip install -r backend/requirements.txt
python backend/app.py
```
```bash
# Windows
python -m venv .venv
.venv\Scripts\activate
pip install -r backend/requirements.txt
python backend/app.py
```

打开 <http://localhost:5050>。端口可用环境变量覆盖：`PORT=8000 python backend/app.py`。

### Docker Compose 部署

> 在**项目根目录**执行

```bash
docker compose build
docker compose up
```

访问地址：<http://localhost:5050>
停止服务：

```bash
docker compose down
```

## 功能

**标准版**

- Web 交互界面 + Flask 后端 API（九个接口）
- 20×20 网格地图，障碍物、机器人实时渲染
- 手动控制（方向键 / 屏幕按钮 / 键盘），越界与撞障被拦截并说明原因
- BFS 路径规划 + 自动巡检
- 虚拟传感器：前 / 左 / 右探测、碰撞检测、越界检测

**拓展功能**

- 科技 HUD 控制台：深色底、发光边框、HUD 角线、状态指示灯
- 路径轨迹：待走路径青色虚线 + 圆点，已走路径浅蓝实线
- 巡检点标记 P1/P2/P3，当前目标点黄色高亮
- 仿真速度滑块 0.5x / 1x / 2x / 4x
- 碰撞、越界、不可达三类告警弹窗，可关闭
- 点击地图添加 / 移除巡检点
- 巡检进度：已完成 / 总点位、剩余步数（严格按登记顺序到达才计数，乱序不追认；增删点位后按前缀匹配重算）
- 俯视小车渲染 + 动效（示警灯 / 轮辐 / 光束呼吸 / 手动单步短闪），行进中位置平滑插值

> 依策划书要求，本项目不实现日志面板。

## 项目结构

```
patrol-sim-std/
├─ .github/workflows/      # GitHub‑CI 流水线配置
├─ frontend/
│  ├─ index.html           # 科技 HUD 单页，Canvas 渲染与按钮交互
│  └─ README.md
├─ backend/
│  ├─ sim/
│  │  ├─ __init__.py
│  │  ├─ path_planner.py
│  │  ├─ robot_sim.py
│  │  └─ sensor_sim.py
│  ├─ test/
│  │  └─ test_sim.py
│  ├─ app.py               # Flask 入口，九个 API + 托管前端
│  ├─ requirements.txt
│  ├─ test/                # 单元测试用例
│  └─ README.md
├─ docs/
│  ├─ 01_项目需求.md
│  ├─ 02_小组分工.md
│  ├─ 03_项目架构与接口.md
│  └─ 04_运行与结业提交.md
├─ README.md
├─ .gitignore
├─ Dockerfile
├─ docker‑compose.yml
└─ AGENTS.md               # 小组协作开发约定
```

## 接口一览

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/map` | 地图尺寸、障碍物、起点、巡检点上限 |
| GET | `/api/robot` | 机器人 x / y / angle / status |
| POST | `/api/move` | 手动移动一格，失败返回 409 + 原因 |
| GET | `/api/points` | 已登记的巡检点（页面加载时恢复） |
| POST | `/api/points` | 登记 / 覆盖巡检点，不启动移动；手动模式的进度来源；巡检进行中改点返回 409 |
| POST | `/api/patrol` | 启动巡检，返回完整路径；`speed` 非四档返回 409 |
| POST | `/api/speed` | 运行中调整巡检速度，从当前格续走 |
| POST | `/api/stop` | 停止巡检 |
| POST | `/api/reset` | 复位：机器人回起点，清空巡检点与进度 |
| GET | `/api/sensor` | 传感器读数 + 巡检进度 |

完整字段见 [docs/03_项目架构与接口.md](docs/03_项目架构与接口.md)。

## 开发

```bash
python backend/app.py            # 默认 0.0.0.0:5050
PORT=8000 python backend/app.py  # 换端口
```

> 目录中不含 `backend/sim/`，需各模块负责人交付后放入才能启动。

## 文档索引

- [02 小组分工](docs/02_小组分工.md)
- [03 项目架构与接口](docs/03_项目架构与接口.md)
- [04 运行与结业提交](docs/04_运行与结业提交.md)
- [后端说明](backend/README.md)
- [前端说明](frontend/README.md)
- [协作开发约定 AGENTS.md](AGENTS.md)
