> **在线演示：** _（部署完成后把实际访问地址填在这里，例如 https://patrol-sim.onrender.com；未部署则写"本地运行，见下方快速开始"）_

# 智能巡检机器人二维仿真系统

一个可以在浏览器里操作的巡检场景：一张 20×20 的二维地图，你可以手动开动机器人，也可以点选巡检点让它按 BFS 规划出的最短路径自动巡检，过程中虚拟传感器持续检测碰撞与越界并弹出告警。

纯软件仿真，不需要实体机器人、GPU 或付费大模型 API。后端 Flask，前端单文件 HTML Canvas，一条 `docker compose` 命令即可完整启动。

## 功能

**标准版**

- Web 交互界面 + Flask 后端 API
- 20×20 二维网格地图，障碍物、机器人实时渲染
- 机器人手动控制（方向键 / 屏幕按钮），状态实时反馈
- BFS 路径规划 + 自动巡检任务
- 虚拟传感器：碰撞检测、越界检测、异常告警

**拓展功能**

- 深色科技控制台 HUD 界面：发光边框、半透明面板、HUD 角线、状态指示灯
- 路径轨迹可视化：待行进路径青色虚线 + 圆点，已走过路径浅蓝色实线
- 巡检点标记 P1/P2/P3，当前目标点黄色高亮
- 仿真速度四档调节：0.5x / 1x / 2x / 4x
- 碰撞、越界、路径规划失败三类半透明告警弹窗，可关闭
- 鼠标点击画布直接添加 / 移除巡检点
- 巡检进度展示：已完成点位 / 总点位、剩余步数

> 依策划书要求，本项目不实现日志面板。

## 快速开始

### Docker（推荐）

```bash
docker compose -f docker/docker-compose.yml up --build
```

打开 <http://localhost:5050>

### 本地运行

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python backend/app.py
```

打开 <http://localhost:5050>。端口用环境变量 `PORT` 覆盖，例如 `PORT=8000 python backend/app.py`。

## 怎么玩

1. 点左侧方向键（或键盘 ↑↓←→）手动移动机器人；撞向障碍物或走出边界会弹告警。
2. 在地图画布上点击任意空格添加巡检点（再点一次移除），最多 6 个。
3. 选好速度档位，点「启动自动巡检」，机器人会按 BFS 规划路径依次走完所有点位。
4. 底部卡片实时显示坐标、运动状态、传感器状态、告警与巡检进度。
5. 「停止任务」立即中断巡检，「复位系统」清空点位与轨迹并让机器人回到起点。

## 项目结构

```
patrol-sim-std/
├─ frontend/              # 单文件 HUD 前端（index.html）
├─ backend/               # Flask 网关 app.py + 模块缺失时的兜底 fallback.py
├─ module_embed_sim/      # 机器人运动仿真
├─ module_ai_sensor/      # 虚拟传感器
├─ module_algorithm/      # BFS 路径规划与巡检调度
├─ docker/                # Dockerfile 与 docker-compose.yml
├─ docs/                  # 接口契约、架构、运行说明、分工
├─ tests/                 # pytest 接口测试
└─ .github/workflows/     # CI
```

## 接口一览

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/map` | 地图尺寸、障碍物、预设巡检点 |
| POST | `/api/move` | 手动移动一格，`{"direction": "up/down/left/right"}` |
| POST | `/api/start_patrol` | 启动巡检，`{"patrol_points": [[x,y]], "speed_multiplier": 1}` |
| GET | `/api/status` | 机器人状态、传感器、进度、路径、轨迹、告警 |
| POST | `/api/stop` | 停止运动 |
| POST | `/api/reset` | 清空任务、轨迹与告警，机器人回起点 |

完整字段定义见 [docs/api_spec.md](docs/api_spec.md)。

## 开发与测试

```bash
pytest -q
```

CI 在每次 push 与 PR 时自动运行测试（`.github/workflows/ci.yml`）。

## 文档

- [接口契约](docs/api_spec.md)
- [系统架构](docs/architecture.md)
- [运行说明](docs/runbook.md)
- [小组分工](docs/team-work.md)

## 技术说明

- 三个业务模块（仿真 / 传感器 / 算法）函数签名严格遵循接口契约，各模块负责人可直接替换同名文件，接口不变。传感器模块已由人工智能组交付，仿真与算法当前为参考实现。
- 任一模块缺失或导入失败时，后端会自动使用内置兜底实现，保证基础版可独立运行。
