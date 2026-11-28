# 模块接口约定 ｜ 标准版 + 拓展 巡检仿真系统

地图固定：20×20 网格，坐标范围 x: 0-19，y: 0-19
障碍物格式：二维列表 `[[x1,y1],[x2,y2]]`

> 本文件是四个模块之间唯一的契约。函数名、入参、返回字段未经全员同意不得修改。

## 1. 嵌入式仿真模块 `module_embed_sim/robot_sim.py`

1. `move_command(direction)`
   - direction 可选字符串：`"up"`, `"down"`, `"left"`, `"right"`
   - 无返回值，修改内部机器人状态

2. `follow_path(points, speed_multiplier)`
   - points：路径点列表 `[[x,y],[x,y]]`
   - speed_multiplier：`0.5 / 1 / 2 / 4` 仿真速度系数
   - 无返回值，机器人按顺序步进行进

3. `stop_move()`
   - 停止机器人运动，task_status 置为 `idle`

4. `get_robot_state()`
   - 返回字典：

```json
{"x": 0, "y": 0, "angle": 0, "is_moving": false, "task_status": "idle"}
```

## 2. 人工智能虚拟传感器模块 `module_ai_sensor/sensor_sim.py`

`sensor_detect(robot_state, obstacle_list)`

- robot_state：`get_robot_state()` 返回的字典
- obstacle_list：障碍物坐标列表

返回字典：

```json
{"sensor_distance": [0,0,0,0], "has_collision": false, "is_out_bound": false, "task_abnormal": false}
```

## 3. 算法模块 `module_algorithm/path_planner.py`

1. `bfs_search(map_size, obstacles, start, end)`
   - 返回 `{"ok": bool, "path": [[x,y],...], "msg": "提示文字"}`；无路可达时 `ok=false`，`msg="无法找到路径"`

2. `patrol_task(map_size, obstacles, patrol_points)`
   - 返回完整拼接好的路径点列表

## 4. 全栈模块

`backend/`（Flask）调用上面三个模块，`frontend/` 提供 HUD 网页。
`GET /api/status` 除返回机器人状态、传感器告警外，额外返回 `patrol_finished`、`patrol_total`、`remain_steps`、`full_path`、`path_index`、`trajectory`、`current_target`、`alerts`。

---

# 附录：补充约定（全栈提出，全员确认后生效）

以下五条是原接口文档未覆盖、但在实现中必须定死的细节。

## 附录 A：障碍物与地图数据由全栈提供

障碍物清单、地图尺寸、预设巡检点统一由 `backend/app.py` 中的 `_build_obstacles()` 与 `PRESET_POINTS` 定义，通过 `GET /api/map` 下发。其他模块不维护地图数据，只接收入参。

## 附录 B：碰撞与越界的告警触发方式

原接口下 `move_command` 禁止穿墙、坐标限制在 0-19，`has_collision` 与 `is_out_bound` 永远不会为真。因此约定：

- **碰撞**：后端在调用 `move_command` 前自行计算目标格，若目标格在障碍物列表中，则不调用移动函数，直接向后端告警队列写入 `collision` 事件。
- **越界**：同理，目标格超出 0-19 时不移动，写入 `out_bound` 事件。
- 两个模块的判定逻辑保持不变，仅由后端负责"尝试移动"这一层的判定。

## 附录 C：`patrol_task` 的起点

`patrol_task` 签名的 `patrol_points` 中，**第一个元素视为起点**，后续元素为巡检目标。后端调用时把机器人当前位置放在首位：

```python
path = patrol_task(20, obstacles, [[robot_x, robot_y]] + patrol_points)
```

## 附录 D：路径规划失败的判定

`patrol_task` 只返回路径列表、没有 `ok` 字段。约定：**返回长度小于 2 即视为规划失败**，后端据此返回 `{"ok": false, "alert": "plan_failed"}` 并弹出告警。

单个点位不可达时，算法模块跳过该点继续拼接，不抛异常。

## 附录 E：`/api/status` 的前后端扩展字段

原文档只约束三个 py 模块之间的调用；`/api/status` 是前端与后端之间的契约，由全栈定义并可扩展。为保证轨迹可视化与进度显示，新增以下字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `full_path` | `[[x,y]]` | 本次巡检的完整路径 |
| `path_index` | int | 已行进到的路径下标 |
| `trajectory` | `[[x,y]]` | 已走过的轨迹点 |
| `current_target` | `[x,y]` 或 null | 当前目标巡检点 |
| `alerts` | `[{id,type,msg,ts}]` | 最近告警事件，`id` 单调递增供前端去重 |
| `patrolling` | bool | 是否处于自动巡检中 |
| `speed` | float | 当前仿真速度系数 |

## 附录 F：巡检中收到手动移动指令

约定为**先停止当前巡检任务，再执行手动移动**。该行为在 `POST /api/move` 内部实现，前端不需要额外处理。
