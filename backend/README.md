# 后端说明

## 接口清单

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/map` | 地图尺寸、障碍物、起点、巡检点上限 |
| GET | `/api/robot` | 机器人 `x` / `y` / `angle` / `status` |
| POST | `/api/move` | `{"direction": "up/down/left/right"}`，失败 409 |
| POST | `/api/patrol` | `{"points": [[x,y]], "speed": 1}`，返回 `plan` |
| POST | `/api/stop` | 停止巡检 |
| GET | `/api/sensor` | 传感器读数 + `progress` |

字段定义见 [../docs/03_项目架构与接口.md](../docs/03_项目架构与接口.md)。

## 依赖版本

| 依赖 | 版本 |
|---|---|
| Python | 3.9+（容器 3.11） |
| Flask | >= 3.0 |

安装：

```bash
pip install -r backend/requirements.txt
```

## 启动命令

```bash
python backend/app.py            # 默认 0.0.0.0:5050
PORT=8000 python backend/app.py  # 换端口
```

## 接口验证示例

```bash
# 地图
curl -s localhost:5050/api/map | python3 -m json.tool

# 手动移动（撞墙时返回 409 与原因）
curl -s -X POST localhost:5050/api/move \
  -H 'Content-Type: application/json' -d '{"direction":"right"}'

# 机器人状态
curl -s localhost:5050/api/robot

# 启动巡检
curl -s -X POST localhost:5050/api/patrol \
  -H 'Content-Type: application/json' -d '{"points":[[3,4]],"speed":2}'

# 传感器读数与进度
curl -s localhost:5050/api/sensor | python3 -m json.tool

# 停止
curl -s -X POST localhost:5050/api/stop
```

## 运行前提

`backend/app.py` 依赖 `backend/sim/` 下的 `robot_sim.py`、`path_planner.py`、`sensor_sim.py`，三个文件由各模块负责人按 `docs/03` 交付后放入该目录。
