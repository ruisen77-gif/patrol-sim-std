> **在线演示：** 本地运行，见下方「快速开始」（未部署公有云，不虚构演示地址）

# 智能巡检机器人二维仿真系统

在浏览器里操作的巡检场景：20×20 二维地图，可手动开动机器人，也可点选巡检点让它按 BFS 规划出的最短路径自动巡检，虚拟传感器持续探测并弹出碰撞、越界告警。

纯软件仿真，不需要实体机器人、GPU 或付费大模型 API。

## 技术选型

| 项 | 选型 |
|---|---|
| 前端 | 原生 HTML / CSS / JavaScript，单文件，无构建步骤 |
| 后端 | Python Flask，六个 REST 接口 |
| 绘图 | Canvas 2D，DPR 适配 |
| 仿真 | 纯 Python 标准库：机器人运动、BFS 寻路、虚拟传感器 |
| CI | GitHub Actions，`compileall` + `pytest` |
| Docker | Dockerfile + docker compose，一条命令启动 |

## 快速开始

### 本地运行

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r backend/requirements.txt
python backend/app.py
```

打开 <http://localhost:5050>。端口可用环境变量覆盖：`PORT=8000 python backend/app.py`。

### Docker

```bash
docker compose up --build
```

打开 <http://localhost:5050>。

## 功能

**标准版**

- Web 交互界面 + Flask 后端 API（六个接口）
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
- 巡检进度：已完成 / 总点位、剩余步数

> 依策划书要求，本项目不实现日志面板。

## 项目结构

```
patrol-sim/
├─ frontend/
│  ├─ index.html          # 科技 HUD 单页，Canvas 渲染与按钮交互
│  ├─ README.md
│  └─ AGENTS.md
├─ backend/
│  ├─ app.py              # Flask 入口，六个 API + 托管前端
│  ├─ sim/
│  │  ├─ __init__.py
│  │  ├─ robot_sim.py     # 嵌入式仿真：坐标 / 朝向 / 行进
│  │  ├─ path_planner.py  # 算法：BFS + 多点串联
│  │  └─ sensor_sim.py    # 人工智能：虚拟传感器
│  ├─ tests/test_sim.py   # CI 运行的测试
│  ├─ requirements.txt
│  ├─ README.md
│  └─ AGENTS.md
├─ docs/                  # 01~04 四份文档
├─ .github/workflows/ci.yml
├─ Dockerfile
├─ docker-compose.yml
├─ README.md
├─ AGENTS.md
└─ .gitignore
```

## 接口一览

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/map` | 地图尺寸、障碍物、起点、巡检点上限 |
| GET | `/api/robot` | 机器人 x / y / angle / status |
| POST | `/api/move` | 手动移动一格，失败返回 409 + 原因 |
| POST | `/api/patrol` | 启动巡检，返回完整路径 |
| POST | `/api/stop` | 停止巡检 |
| GET | `/api/sensor` | 传感器读数 + 巡检进度 |

完整字段见 [docs/03_项目架构与接口.md](docs/03_项目架构与接口.md)。

## 开发与测试

```bash
pytest backend/tests -q
```

CI 在每次 push 与 PR 时自动运行编译检查与测试（`.github/workflows/ci.yml`）。

## 文档索引

- [01 项目需求](docs/01_项目需求.md)
- [02 小组分工](docs/02_小组分工.md)
- [03 项目架构与接口](docs/03_项目架构与接口.md)
- [04 运行与结业提交](docs/04_运行与结业提交.md)
- [后端说明](backend/README.md)
- [前端说明](frontend/README.md)
