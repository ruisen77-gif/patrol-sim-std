# -*- coding: utf-8 -*-
"""
sim/sensor_sim.py —— 智能巡检机器人二维仿真系统 · 人工智能模块（虚拟传感器）

【模块职责】
    纯感知模块：根据机器人当前位置与朝向，模拟车载传感器探测前、左、右
    三格是否有障碍，并判断机器人是否压在障碍物上、是否越出地图边界。
    不控制机器人、不规划路径、不写界面、不访问网络。由 backend/app.py 调用。

【数据约定】（与 docs/03_项目架构与接口.md 保持一致）
    * 地图为 size×size 网格（本项目 20×20），坐标原点在左上角 (0,0)，
      x 向右递增，y 向下递增，单位为“格”，取值范围 0 ~ size-1；
    * 机器人朝向 angle 取 0 向右、90 向下、180 向左、270 向上；
      左、右按机器人自身视角判断：朝右时左手边是上（y 减小）；
    * 障碍物用二维整数坐标列表表示，如 [[5,5],[5,6],[12,3]]，
      落在这些格子上的位置不可通行；
    * 探测格若落在地图外，同样视为被阻挡（等同墙体）。

【函数契约】（已写入 docs/03，改动须全员同意）
    detect(state, obstacles, size) -> dict

    正常返回，且固定只有这五个字段：
        {"front_blocked": bool,   # 正前方一格是否被阻挡
         "left_blocked":  bool,   # 左手边一格是否被阻挡
         "right_blocked": bool,   # 右手边一格是否被阻挡
         "collision":     bool,   # 当前所在格是否正好压在障碍物上
         "out_of_bound":  bool}   # 当前坐标是否越出 0 ~ size-1 范围

    输入非法（不是字典、缺少 x/y/angle、坐标不是整数、size 非法等）时：
        不抛异常，返回五个字段全 False，并额外带一个 "msg" 说明原因。

【供全栈同学（app.py）接入的写法】
    readings = sensor_sim.detect(robot_sim.get_state(), obstacles, 20)
    # readings 直接合并进 GET /api/sensor 的响应即可；
    # collision / out_of_bound 为 True 时前端弹告警框，
    # front_blocked 可用于提示“前方有障碍，无法前进”。
"""

# 朝向 -> 正前方的单位向量（y 向下递增，故 90 度朝下）
_FORWARD_VECTORS = {
    0: (1, 0),     # 向右
    90: (0, 1),    # 向下
    180: (-1, 0),  # 向左
    270: (0, -1),  # 向上
}

_FIELDS = ("front_blocked", "left_blocked", "right_blocked", "collision", "out_of_bound")


def _as_int(value, name):
    """把输入转成整数，失败时抛 ValueError（由 detect 统一兜住，不往外抛）。"""
    if isinstance(value, bool):
        raise ValueError("%s 不能是布尔值：%r" % (name, value))
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if value.is_integer():
            return int(value)
        raise ValueError("%s 必须是整数，实际为 %r" % (name, value))
    if isinstance(value, str):
        text = value.strip()
        if text.lstrip("+-").isdigit():
            return int(text)
    raise ValueError("%s 必须是整数，实际为 %r" % (name, value))


def _normalize_angle(angle):
    """把任意角度归一到 0 / 90 / 180 / 270（就近取整，360 度视为 0 度）。"""
    value = _as_int(angle, "angle")
    return int(round(value / 90.0)) * 90 % 360


def _normalize_obstacles(obstacles, size):
    """障碍物归一化成 {(x, y), ...} 集合；格式不对的条目跳过，不抛异常。"""
    blocked = set()
    for cell in obstacles or []:
        try:
            x, y = _as_int(cell[0], "障碍物 x"), _as_int(cell[1], "障碍物 y")
        except (TypeError, IndexError, KeyError, ValueError):
            continue
        # 地图外的障碍物不参与判断：探测格在地图外本来就算阻挡
        if 0 <= x < size and 0 <= y < size:
            blocked.add((x, y))
    return blocked


def _parse_state(state):
    """校验并取出 (x, y, angle)，非法则抛 ValueError，由 detect 转成全 False + msg。"""
    if not isinstance(state, dict):
        raise ValueError("state 必须是字典（robot_sim.get_state() 的返回值），"
                         "实际为 %s" % type(state).__name__)
    missing = [key for key in ("x", "y", "angle") if key not in state]
    if missing:
        raise ValueError("state 缺少字段 %s" % missing)
    return (_as_int(state["x"], "x"),
            _as_int(state["y"], "y"),
            _normalize_angle(state["angle"]))


def _neighbors(x, y, angle):
    """按朝向推算前、左、右三个相邻格的坐标。

    设前方单位向量 f = (dx, dy)，则：
        前方 = (x + dx, y + dy)
        左手 = (x + dy, y - dx)   # 朝右时左手边在上方
        右手 = (x - dy, y + dx)   # 朝右时右手边在下方
    以朝下（0, 1）为例，左手边为 (1, 0) 即东侧，与真人面向南时左手朝东一致。
    """
    dx, dy = _FORWARD_VECTORS[angle]
    return {
        "front": (x + dx, y + dy),
        "left": (x + dy, y - dx),
        "right": (x - dy, y + dx),
    }


def _blocked(cell, blocked, size):
    """探测格是否被阻挡：越出地图算阻挡，落在障碍物上也算阻挡。"""
    x, y = cell
    if not (0 <= x < size and 0 <= y < size):
        return True
    return (x, y) in blocked


def detect(state, obstacles, size=20):
    """
    虚拟传感器检测入口（对外唯一接口，函数名/入参/返回字段禁止改动）。

    参数：
        state     : dict，至少包含 x、y、angle，如 robot_sim.get_state() 的返回值
        obstacles : 障碍物坐标列表，如 [[5,5],[5,6],[12,3]]
        size      : int，正方形地图边长，本项目为 20

    返回（dict，正常时严格五个字段）：
        front_blocked / left_blocked / right_blocked / collision / out_of_bound

    输入非法时返回五个字段全 False，并追加 "msg" 说明原因，不抛异常。

    示例：
        >>> detect({"x": 5, "y": 5, "angle": 0}, [[6, 5]], 20)["front_blocked"]
        True
        >>> detect({"x": -1, "y": 5, "angle": 0}, [], 20)["out_of_bound"]
        True
    """
    result = dict.fromkeys(_FIELDS, False)

    try:
        size = _as_int(size, "size")
        if size <= 0:
            raise ValueError("size 必须是正整数，实际为 %r" % size)
        x, y, angle = _parse_state(state)
    except ValueError as exc:
        result["msg"] = str(exc)
        return result

    blocked = _normalize_obstacles(obstacles, size)

    # 机器人当前所在格是否压障碍、是否越界：先判断自身状态
    result["collision"] = (x, y) in blocked
    result["out_of_bound"] = not (0 <= x < size and 0 <= y < size)

    # 三方向探测；即使机器人已经越界，仍按格子几何给出读数，方便排查
    cells = _neighbors(x, y, angle)
    for name in ("front", "left", "right"):
        result[name + "_blocked"] = _blocked(cells[name], blocked, size)

    return result


# ======================================================================
# 自测代码：直接运行 python sim/sensor_sim.py 即可
# 覆盖：空旷处、正前方有障碍、三面被围、边界角落、越界坐标五种场景，
#       另加朝向换算、非法输入、感知随位置变化三项检查。
# ======================================================================

def _show(name, state, obstacles, size=20, expect=None):
    """跑一个场景并打印结果，返回是否与期望一致。"""
    readings = detect(state, obstacles, size)
    ok = True
    if expect:
        for key, value in expect.items():
            if readings.get(key) is not value:
                ok = False
    print("  机器人 (x=%s, y=%s, angle=%s)  障碍物 %s"
          % (state.get("x"), state.get("y"), state.get("angle"), obstacles))
    print("  读数 -> front=%s left=%s right=%s collision=%s out_of_bound=%s"
          % (readings.get("front_blocked"), readings.get("left_blocked"),
             readings.get("right_blocked"), readings.get("collision"),
             readings.get("out_of_bound")))
    if "msg" in readings:
        print("  msg -> %s" % readings["msg"])
    print("[%s] %s" % ("通过" if ok else "失败", name))
    return ok


def _self_test():
    print("=" * 66)
    print("sim/sensor_sim.py 自测：虚拟传感器三方向探测 + 碰撞 + 越界")
    print("=" * 66)
    results = []

    # ---- 场景一：空旷处，三方向都不阻挡 ----
    print("\n场景一：空旷处（20×20 无障碍，机器人在 [5,5] 朝右）")
    results.append(_show("场景一 空旷处", {"x": 5, "y": 5, "angle": 0}, [], expect={
        "front_blocked": False, "left_blocked": False, "right_blocked": False,
        "collision": False, "out_of_bound": False}))

    # ---- 场景二：正前方有障碍 ----
    print("\n场景二：正前方有障碍（[5,5] 朝右，前方 [6,5] 有障碍）")
    results.append(_show("场景二 正前方有障碍", {"x": 5, "y": 5, "angle": 0},
                         [[6, 5]], expect={
                             "front_blocked": True, "left_blocked": False,
                             "right_blocked": False, "collision": False}))

    # ---- 场景三：三面被围 ----
    print("\n场景三：三面被围（[5,5] 朝下，前 [5,6]、左 [6,5]、右 [4,5] 均堵住）")
    results.append(_show("场景三 三面被围", {"x": 5, "y": 5, "angle": 90},
                         [[5, 6], [6, 5], [4, 5]], expect={
                             "front_blocked": True, "left_blocked": True,
                             "right_blocked": True, "collision": False}))

    # ---- 场景四：边界角落，地图外视为阻挡 ----
    print("\n场景四：边界角落（[0,0] 朝上，前方与左手边都在地图外）")
    results.append(_show("场景四 边界角落", {"x": 0, "y": 0, "angle": 270}, [],
                         expect={"front_blocked": True, "left_blocked": True,
                                 "right_blocked": False, "collision": False,
                                 "out_of_bound": False}))

    # ---- 场景五：越界坐标，不抛异常且 out_of_bound 为 True ----
    print("\n场景五：越界坐标（机器人在 [-1,5]，已越出左边界）")
    results.append(_show("场景五 越界坐标", {"x": -1, "y": 5, "angle": 0}, [],
                         expect={"out_of_bound": True, "collision": False}))
    print("\n场景五补：越界坐标（[20,19]，已越出右边界）")
    results.append(_show("场景五补 越界坐标", {"x": 20, "y": 19, "angle": 180}, [],
                         expect={"out_of_bound": True}))

    # ---- 附加一：四个朝向的前/左/右换算都必须正确 ----
    print("\n附加一：四个朝向前/左/右换算（机器人在 [10,10]，无遮挡）")
    # 期望：0 度 前(11,10) 左(10,9) 右(10,11)；90 度 前(10,11) 左(11,10) 右(9,10)
    expect_cells = {
        0: ((11, 10), (10, 9), (10, 11)),
        90: ((10, 11), (11, 10), (9, 10)),
        180: ((9, 10), (10, 11), (10, 9)),
        270: ((10, 9), (9, 10), (11, 10)),
    }
    ok_cells = True
    for angle, (front, left, right) in expect_cells.items():
        cells = _neighbors(10, 10, angle)
        got = (cells["front"], cells["left"], cells["right"])
        if got != (front, left, right):
            ok_cells = False
        print("  angle=%3d  前=%s  左=%s  右=%s" % (angle, cells["front"], cells["left"], cells["right"]))
    print("[%s] 附加一 朝向换算" % ("通过" if ok_cells else "失败"))
    results.append(ok_cells)

    # ---- 附加二：非法输入必须全 False 且带 msg，不抛异常 ----
    print("\n附加二：非法输入不抛异常，返回全 False 并带 msg")
    bad_cases = [
        ("state 不是字典", "not-a-dict", [], 20),
        ("state 缺 y 字段", {"x": 1, "angle": 0}, [], 20),
        ("x 不是整数", {"x": "abc", "y": 1, "angle": 0}, [], 20),
        ("size 非法", {"x": 1, "y": 1, "angle": 0}, [], 0),
        ("angle 非法", {"x": 1, "y": 1, "angle": "up"}, [], 20),
    ]
    ok_bad = True
    for name, state, obstacles, size in bad_cases:
        try:
            readings = detect(state, obstacles, size)
        except Exception as exc:                     # 不允许抛出任何异常
            ok_bad = False
            print("  [失败] %s：抛出了 %s" % (name, type(exc).__name__))
            continue
        all_false = all(readings[key] is False for key in _FIELDS)
        has_msg = bool(readings.get("msg"))
        if not (all_false and has_msg):
            ok_bad = False
        print("  %-12s -> 全 False=%s，msg=%s" % (name, all_false, readings.get("msg")))
    print("[%s] 附加二 非法输入处理" % ("通过" if ok_bad else "失败"))
    results.append(ok_bad)

    # ---- 附加三：感知结果随位置变化 ----
    print("\n附加三：同一障碍物，机器人换位置后读数随之变化")
    wall = [[6, 5]]
    before = detect({"x": 4, "y": 5, "angle": 0}, wall)
    after = detect({"x": 5, "y": 5, "angle": 0}, wall)
    changed = (before["front_blocked"] is False) and (after["front_blocked"] is True)
    print("  [4,5] 朝右 -> front_blocked=%s" % before["front_blocked"])
    print("  [5,5] 朝右 -> front_blocked=%s" % after["front_blocked"])
    print("[%s] 附加三 读数随位置变化" % ("通过" if changed else "失败"))
    results.append(changed)

    print("\n" + "=" * 66)
    passed, total = sum(results), len(results)
    print("自测结果：%d/%d 个场景通过" % (passed, total))
    print("=" * 66)
    return passed == total


if __name__ == "__main__":
    import sys
    sys.exit(0 if _self_test() else 1)


__all__ = ["detect"]
