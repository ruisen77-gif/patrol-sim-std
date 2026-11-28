# -*- coding: utf-8 -*-
"""人工智能虚拟传感器模块：根据位置与朝向探测前、左、右三格。

对外函数（全组契约）：
    detect(state, obstacles, size=20) -> dict
        {"front_blocked": bool, "left_blocked": bool, "right_blocked": bool,
         "collision": bool, "out_of_bound": bool}
        输入非法时额外带 "msg"，且各项为 False，不抛异常。

判定口径：
    - 前 / 左 / 右：按 angle（0 右、90 下、180 左、270 上）换算出三个相邻格；
      相邻格越界或位于障碍物上都算 blocked。
    - collision：当前坐标恰好落在障碍物上。
    - out_of_bound：当前坐标超出 0-19。
本模块只做感知，不移动机器人、不涉及界面。
"""

MAP_SIZE = 20

# angle -> (前方 dx, dy)；左侧为前方逆时针 90°，即 (dy, -dx)
VECTORS = {
    0: (1, 0),
    90: (0, 1),
    180: (-1, 0),
    270: (0, -1),
}


def _point_set(obstacles):
    points = set()
    for item in obstacles or []:
        try:
            points.add((int(item[0]), int(item[1])))
        except (TypeError, ValueError, IndexError, KeyError):
            continue
    return points


def _empty(msg):
    return {
        "front_blocked": False,
        "left_blocked": False,
        "right_blocked": False,
        "collision": False,
        "out_of_bound": False,
        "msg": msg,
    }


def _blocked(x, y, size, occupied):
    return not (0 <= x < size and 0 <= y < size) or (x, y) in occupied


def detect(state, obstacles, size=MAP_SIZE):
    if not isinstance(state, dict):
        return _empty("robot_state 必须是字典")

    try:
        x = int(state["x"])
        y = int(state["y"])
        angle = int(state.get("angle", 0))
    except (KeyError, TypeError, ValueError):
        return _empty("robot_state 缺少 x / y / angle")

    size = int(size) if size else MAP_SIZE
    angle = angle % 360
    if angle not in VECTORS:
        angle = 0

    occupied = _point_set(obstacles)
    out_of_bound = not (0 <= x < size and 0 <= y < size)
    collision = (x, y) in occupied

    fx, fy = VECTORS[angle]
    lx, ly = fy, -fx
    rx, ry = -fy, fx

    return {
        "front_blocked": _blocked(x + fx, y + fy, size, occupied),
        "left_blocked": _blocked(x + lx, y + ly, size, occupied),
        "right_blocked": _blocked(x + rx, y + ry, size, occupied),
        "collision": collision,
        "out_of_bound": out_of_bound,
    }


def _self_test():
    obstacles = [[6, 5], [5, 4], [5, 6], [10, 10]]
    size = 20

    print("=" * 62)
    print("sensor_sim 自测：障碍物 %s" % obstacles)
    print("=" * 62)

    ok = True

    def check(name, state, expect, angle_note=""):
        nonlocal ok
        result = detect(state, obstacles, size)
        got = {k: result[k] for k in ("front_blocked", "left_blocked", "right_blocked",
                                      "collision", "out_of_bound")}
        passed = got == expect
        ok = ok and passed
        print("[%s] %s%s  %s" % ("PASS" if passed else "FAIL", name, angle_note, got))

    empty = {"front_blocked": False, "left_blocked": False, "right_blocked": False,
             "collision": False, "out_of_bound": False}

    check("空旷处", {"x": 12, "y": 8, "angle": 0}, empty)

    check("正前方有障碍", {"x": 5, "y": 3, "angle": 90}, dict(empty, front_blocked=True),
          "（朝下，前方 (5,4) 为障碍）")

    check("左侧有障碍", {"x": 6, "y": 6, "angle": 0}, dict(empty, left_blocked=True),
          "（朝右，左侧 (6,5) 为障碍）")

    three = {"front_blocked": True, "left_blocked": True, "right_blocked": True,
             "collision": False, "out_of_bound": False}
    check("三面被围", {"x": 5, "y": 5, "angle": 0}, three,
          "（前 (6,5)、左 (5,4)、右 (5,6) 均为障碍）")

    corner = {"front_blocked": False, "left_blocked": True, "right_blocked": False,
              "collision": False, "out_of_bound": False}
    check("地图角落", {"x": 0, "y": 0, "angle": 0}, corner, "（左侧越界、前方与右侧空地）")

    out = {"front_blocked": False, "left_blocked": True, "right_blocked": True,
           "collision": False, "out_of_bound": True}
    check("越界坐标", {"x": -1, "y": 5, "angle": 0}, out)

    hit = dict(empty, collision=True)
    check("坐标压障碍", {"x": 5, "y": 4, "angle": 0}, hit)

    bad = detect({"x": "a", "y": 1}, obstacles, size)
    passed = set(bad.keys()) == {"front_blocked", "left_blocked", "right_blocked",
                                 "collision", "out_of_bound", "msg"} and "msg" in bad
    ok = ok and passed
    print("[%s] 非法输入不抛异常  %s" % ("PASS" if passed else "FAIL", bad))

    print("-" * 62)
    print("自测结论：%s" % ("全部通过" if ok else "存在失败用例"))
    return ok


if __name__ == "__main__":
    import sys

    sys.exit(0 if _self_test() else 1)
