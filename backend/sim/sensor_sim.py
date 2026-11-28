# -*- coding: utf-8 -*-
"""虚拟传感器仿真模块（角色B：人工智能模块）

严格按照 docs/api_spec.md 的接口约定实现：

    函数：sensor_detect(robot_state, obstacle_list)
    robot_state  ：robot_sim.get_robot_state() 返回的字典
                   {"x":int,"y":int,"angle":0,"is_moving":bool,"task_status":"idle"/"moving"}
    obstacle_list：障碍物坐标二维列表 [[x,y], ...]

    返回字典：
    {"sensor_distance":[0,0,0,0],
     "has_collision":bool,
     "is_out_bound":bool,
     "task_abnormal":bool}

说明：
1. 只使用 Python 标准库，不依赖任何第三方包。
2. 本模块只做“感知”，不移动机器人、不修改机器人状态、不涉及界面。
3. 地图固定 20×20 网格，坐标合法范围 x: 0-19，y: 0-19。
4. sensor_distance 在标准版中固定返回 [0,0,0,0]（接口占位），
   后续如需扩展可在此处计算四方向探测距离，不影响返回字段名。
"""

from typing import Dict, Iterable, List, Sequence, Tuple, Union

# 地图边长：20×20 网格，合法坐标 0-19
MAP_SIZE = 20

# 标准版接口规定的传感器距离占位值（上、下、左、右）
SENSOR_DISTANCE_PLACEHOLDER: List[int] = [0, 0, 0, 0]

# 坐标点支持三种写法：[x, y]、(x, y)、{"x": .., "y": ..}
PointLike = Union[Sequence[int], Dict[str, int]]


def _to_point(point: PointLike) -> Tuple[int, int]:
    """把各种写法的坐标统一转换成 (x, y) 整数二元组。"""
    if isinstance(point, dict):
        if "x" not in point or "y" not in point:
            raise ValueError("坐标字典必须同时包含 x 和 y 字段：{!r}".format(point))
        return int(point["x"]), int(point["y"])

    if isinstance(point, (list, tuple)) and len(point) >= 2:
        return int(point[0]), int(point[1])

    raise ValueError("无法解析坐标点，应为 [x, y] 或 {{\"x\": .., \"y\": ..}}：{!r}".format(point))


def _to_obstacle_set(obstacle_list: Iterable[PointLike]) -> set:
    """把障碍物列表转换成 {(x, y), ...} 集合，便于 O(1) 判断碰撞。"""
    if not obstacle_list:
        return set()

    obstacles = set()
    for item in obstacle_list:
        obstacles.add(_to_point(item))
    return obstacles


def _read_robot_position(robot_state: Dict[str, int]) -> Tuple[int, int]:
    """从机器人状态字典中取出 (x, y)，字段缺失时给出明确报错。"""
    if not isinstance(robot_state, dict):
        raise TypeError("robot_state 必须是字典，实际收到：{}".format(type(robot_state).__name__))

    missing = [key for key in ("x", "y") if key not in robot_state]
    if missing:
        raise KeyError("robot_state 缺少字段 {}，请检查是否由 get_robot_state() 直接传入".format(missing))

    return int(robot_state["x"]), int(robot_state["y"])


def is_out_bound(x: int, y: int, map_size: int = MAP_SIZE) -> bool:
    """判断坐标是否越界：x<0 或 x>=map_size 或 y<0 或 y>=map_size。"""
    return x < 0 or x >= map_size or y < 0 or y >= map_size


def is_collision(x: int, y: int, obstacles: set) -> bool:
    """判断机器人所在格子是否与障碍物重合。"""
    return (x, y) in obstacles


def sensor_detect(robot_state: Dict[str, int], obstacle_list: Iterable[PointLike]) -> Dict[str, object]:
    """虚拟传感器检测入口（对外唯一接口，函数名/入参/返回字段禁止改动）。

    参数
    ----
    robot_state  : dict，robot_sim.get_robot_state() 的返回值，至少包含 x、y。
    obstacle_list: 障碍物坐标列表 [[x,y], ...]，允许为空列表或 None。

    返回
    ----
    dict，字段严格为 sensor_distance / has_collision / is_out_bound / task_abnormal。
    """
    x, y = _read_robot_position(robot_state)
    obstacles = _to_obstacle_set(obstacle_list)

    has_collision = is_collision(x, y, obstacles)
    out_bound = is_out_bound(x, y, MAP_SIZE)

    return {
        # 标准版固定占位；四方向顺序为 上、下、左、右
        "sensor_distance": list(SENSOR_DISTANCE_PLACEHOLDER),
        "has_collision": has_collision,
        "is_out_bound": out_bound,
        # 任一异常即视为任务异常，正常空地时为 False（符合“默认 False”的约定）
        "task_abnormal": has_collision or out_bound,
    }


# ---------------------------------------------------------------------------
# 自测代码：直接运行 python sensor_sim.py 即可，覆盖碰撞 / 越界 / 正常空地三类场景
# ---------------------------------------------------------------------------

def _build_robot_state(x: int, y: int) -> Dict[str, object]:
    """构造一个与 get_robot_state() 完全一致结构的机器人状态字典。"""
    return {"x": x, "y": y, "angle": 0, "is_moving": False, "task_status": "idle"}


def _self_test() -> bool:
    """多组场景自测，打印每个场景的输入与传感器返回结果。"""
    obstacles = [[3, 3], [4, 3], [3, 4], [10, 10], [0, 0]]

    # (场景名, 机器人坐标, 期望 has_collision, 期望 is_out_bound)
    cases = [
        ("正常空地", (5, 5), False, False),
        ("正常空地-地图右下角", (19, 19), False, False),
        ("碰撞-障碍物中心", (3, 3), True, False),
        ("碰撞-障碍物边缘", (10, 10), True, False),
        ("碰撞-地图原点有障碍", (0, 0), True, False),
        ("越界-负坐标", (-1, 5), False, True),
        ("越界-x 超出右边界", (20, 5), False, True),
        ("越界-y 超出下边界", (5, 20), False, True),
        ("越界-负坐标且压障碍", (-1, 3), False, True),
    ]

    print("=" * 78)
    print("虚拟传感器模块 sensor_sim.py 自测")
    print("地图：{}×{} 网格，坐标 0-{}；障碍物：{}".format(MAP_SIZE, MAP_SIZE, MAP_SIZE - 1, obstacles))
    print("=" * 78)

    all_pass = True
    for name, (x, y), expect_collision, expect_bound in cases:
        state = _build_robot_state(x, y)
        result = sensor_detect(state, obstacles)

        ok = (
            result["has_collision"] is expect_collision
            and result["is_out_bound"] is expect_bound
            and result["sensor_distance"] == [0, 0, 0, 0]
            and result["task_abnormal"] is (expect_collision or expect_bound)
        )
        all_pass = all_pass and ok

        print("\n[{}] {}".format("PASS" if ok else "FAIL", name))
        print("  机器人状态: x={}, y={}".format(x, y))
        print("  传感器返回: {}".format(result))
        print("  期望结果  : has_collision={}, is_out_bound={}, task_abnormal={}".format(
            expect_collision, expect_bound, expect_collision or expect_bound))

    # 兼容性检查：空障碍物列表 / None 不应报错
    print("\n[兼容性] 障碍物为空列表与 None 时不应报错")
    for empty in ([], None):
        result = sensor_detect(_build_robot_state(5, 5), empty)
        ok = result["has_collision"] is False and result["is_out_bound"] is False
        all_pass = all_pass and ok
        print("  obstacle_list={!r:<5} -> {} ({})".format(empty, result, "PASS" if ok else "FAIL"))

    # 字段完整性检查：返回字典必须严格是这 4 个字段
    required_keys = {"sensor_distance", "has_collision", "is_out_bound", "task_abnormal"}
    result = sensor_detect(_build_robot_state(5, 5), obstacles)
    keys_ok = set(result.keys()) == required_keys
    all_pass = all_pass and keys_ok
    print("\n[字段检查] 返回字典字段 = {} ({})".format(sorted(result.keys()), "PASS" if keys_ok else "FAIL"))

    print("\n" + "=" * 78)
    print("自测结论：{}".format("全部通过" if all_pass else "存在失败用例，请检查"))
    print("=" * 78)
    return all_pass


if __name__ == "__main__":
    import sys

    sys.exit(0 if _self_test() else 1)
