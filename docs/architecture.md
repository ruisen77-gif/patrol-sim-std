# 系统架构文档

## 模块列表

1. **全栈模块**：Flask 后端网关 + Web 前端科技控制台，作为系统对外入口，调用其他业务模块；
2. **嵌入式仿真模块**：维护机器人本体状态，处理手动、路径跟随运动逻辑；
3. **人工智能虚拟传感器模块**：接收机器人状态与地图障碍物，输出传感器数据与异常告警；
4. **算法模块**：BFS 路径规划、巡检任务调度，输出路径序列；
5. **运维模块**：版本管理、容器打包、部署、项目文档维护。

## 模块依赖

```
frontend/index.html
        │  HTTP(JSON) 轮询 /api/status
        ▼
backend/app.py  ──import──►  module_embed_sim/robot_sim.py
        │      ──import──►  module_ai_sensor/sensor_sim.py
        │      ──import──►  module_algorithm/path_planner.py
        │
        └── 持有全局任务状态：FULL_PATH / PATH_INDEX / TRAJECTORY / PATROL_POINTS / ALERTS
```

后端是唯一有状态的进程：三个业务模块都是函数 + 模块级变量，不感知"当前巡检任务"，任务进度由后端统一维护。

## 数据流

### 手动控制模式

```
前端方向键 → POST /api/move → 后端预判目标格（障碍/越界）
        → robot_sim.move_command() → 更新坐标
        → 前端轮询 GET /api/status → 调用 sensor_detect() → 渲染画布与告警弹窗
```

### 自动巡检模式

```
① 用户点击地图添加巡检点（或使用预设点）
② 前端 POST /api/start_patrol {patrol_points, speed_multiplier}
③ 后端调用 path_planner.patrol_task() 生成完整路径
④ 后端起后台线程调用 robot_sim.follow_path(points, speed)
⑤ 后端每 20ms 采样机器人位置，推进 PATH_INDEX 与 TRAJECTORY
⑥ 前端轮询 /api/status 渲染路径三态、巡检标记、进度、告警
⑦ 全部点位走完后机器人回到 idle
```

## 关键设计决策

| 决策 | 原因 |
|---|---|
| `follow_path` 在后台线程执行 | 该函数内部 sleep，同步调用会阻塞 Flask，使 `/api/status` 超时 |
| 碰撞/越界由后端预判 | 原接口下机器人永远不会进入障碍或越界，告警无法触发（见 api_spec 附录 B） |
| 任务进度由路径推导 | `get_robot_state()` 不含进度字段，后端用当前坐标反查 `FULL_PATH` 下标 |
| 三个模块缺失时用 mock | 任一模块未交付时系统仍可启动，保证基础版可演示 |

## 地图约定

20×20 网格，坐标 x、y 取值 0-19；障碍物格式 `[[x,y],[x,y]]`。
