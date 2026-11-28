# AGENTS.md

## 项目目标与当前范围

基于 Web 的智能巡检机器人二维仿真系统：基础版全部通过 + 标准版四项全部实现 + 进阶版七项拓展。

## 目录职责

| 目录 | 放什么 |
|---|---|
| `frontend/` | 只放页面：`index.html` 与前端说明 |
| `backend/` | Flask 入口 `app.py`、`sim/` 三个仿真模块、测试、依赖 |
| `backend/sim/` | 纯逻辑模块，只依赖标准库，不导入 Flask |
| `docs/` | 01–04 四份文档，任何接口与口径改动先改这里 |
| `.agents/skills/` | 可复用的 AI 提示词与技能说明 |
| `.github/workflows/` | CI 配置 |

## 开发约定

- 坐标原点在地图左上角，x 向右、y 向下递增，20×20，合法坐标 0–19。
- 机器人字段：`x`、`y`、`angle`（0 右 / 90 下 / 180 左 / 270 上）、`status`（`idle` / `moving` / `finished`）。
- 接口失败统一 HTTP 409 + `{"ok": false, "msg": 原因}`。
- 接口返回格式以 `docs/03_项目架构与接口.md` 为准，不得单方面增删字段。

## 常用命令

```bash
python backend/app.py              # 启动后端（默认 5050）
pytest backend/tests -q            # 跑测试
docker compose up --build          # 容器启动
python backend/sim/robot_sim.py    # 单模块自测
```

## 协作边界

| 角色 | 维护文件 |
|---|---|
| 全栈 | `frontend/`、`backend/app.py`、根 `README.md` |
| 运维 | `Dockerfile`、`docker-compose.yml`、`.github/`、三份 `AGENTS.md` |
| 算法 | `backend/sim/path_planner.py` |
| 人工智能 | `backend/sim/sensor_sim.py` |
| 嵌入式仿真 | `backend/sim/robot_sim.py` |

禁止跨模块直接改他人代码；确需改接口时先在 `docs/03` 提出并全员同意。
