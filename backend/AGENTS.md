# backend/AGENTS.md

## 边界

- `app.py` 只做编排：解析请求、调用 `sim/` 三个模块、拼装响应。业务算法不得写进网关。
- `sim/` 下的三个模块只依赖 Python 标准库，不得 import Flask、不得读写文件、不得访问网络。
- 地图数据（障碍物、尺寸）只在 `app.py` 定义一次，通过 `robot_sim.set_map()` 注入，其他模块只接收入参。

## 模块契约

| 模块 | 函数 |
|---|---|
| `sim/robot_sim.py` | `set_map`、`move`、`get_state`、`follow`、`stop`、`reset` |
| `sim/path_planner.py` | `bfs`、`patrol` |
| `sim/sensor_sim.py` | `detect` |

函数名、入参、返回字段改动须全员同意，并同步 `docs/03`。

## 自测

```bash
python backend/sim/robot_sim.py
python backend/sim/path_planner.py
python backend/sim/sensor_sim.py
pytest backend/tests -q
```
