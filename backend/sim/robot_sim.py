# -*- coding: utf-8 -*-
"""嵌入式仿真模块：维护机器人坐标、朝向与状态。

对外函数（全组契约）：
    set_map(obstacles, size)      注入地图，后端启动时调用一次
    move(direction)   -> dict     手动移动一格，成功 {"ok": True}，失败 {"ok": False, "msg": 原因}
    get_state()       -> dict     {"x", "y", "angle", "status"}
    follow(points, speed) -> None 按点位序列行进，内部线程推进，调用即返回
    stop()            -> None     终止行进，status 置回 idle
    reset()           -> None     回到起点并清空运动状态（测试与复位使用）

约定：angle 取 0 向右、90 向下、180 向左、270 向上；status 取 idle / moving / finished。
本模块只做运动，不寻路、不感知、不涉及界面。
"""

import threading
import time

MAP_SIZE = 20
BASE_INTERVAL = 0.2  # speed 为 1 时每走一格停顿的秒数

DIRECTIONS = {
    "up": (0, -1, 270),
    "down": (0, 1, 90),
    "left": (-1, 0, 180),
    "right": (1, 0, 0),
}

_lock = threading.RLock()
_state = {"x": 0, "y": 0, "angle": 0, "status": "idle"}
_size = MAP_SIZE
_obstacles = set()
_stop_event = threading.Event()
_worker = None


def set_map(obstacles, size=MAP_SIZE):
    global _size, _obstacles
    with _lock:
        _size = int(size)
        _obstacles = {(int(o[0]), int(o[1])) for o in obstacles or []}


def get_state():
    with _lock:
        return {
            "x": int(_state["x"]),
            "y": int(_state["y"]),
            "angle": int(_state["angle"]),
            "status": _state["status"],
        }


def move(direction):
    if direction not in DIRECTIONS:
        return {"ok": False, "msg": "方向必须是 up/down/left/right"}

    stop()  # 巡检途中收到手动指令：先停车再走一格

    dx, dy, angle = DIRECTIONS[direction]
    with _lock:
        nx, ny = _state["x"] + dx, _state["y"] + dy
        if not (0 <= nx < _size and 0 <= ny < _size):
            return {"ok": False, "msg": "前方 (%d, %d) 越出地图边界" % (nx, ny)}
        if (nx, ny) in _obstacles:
            return {"ok": False, "msg": "前方 (%d, %d) 存在障碍物" % (nx, ny)}
        _state.update(x=nx, y=ny, angle=angle)
        return {"ok": True}


def follow(points, speed=1):
    global _worker
    stop()

    path = [[int(p[0]), int(p[1])] for p in points or []]
    if len(path) < 2:
        return
    try:
        multiplier = float(speed)
    except (TypeError, ValueError):
        multiplier = 1.0
    if multiplier <= 0:
        multiplier = 1.0

    global _stop_event
    _stop_event = threading.Event()
    event = _stop_event
    with _lock:
        _state["status"] = "moving"

    _worker = threading.Thread(target=_run, args=(path, multiplier, event), daemon=True)
    _worker.start()


def _run(path, multiplier, event):
    interval = BASE_INTERVAL / multiplier
    stopped = False
    try:
        for i in range(1, len(path)):
            if event.is_set():
                stopped = True
                break
            x, y = path[i]
            with _lock:
                dx, dy = x - _state["x"], y - _state["y"]
                if dx or dy:
                    _state["angle"] = _angle_of(dx, dy)
                _state["x"], _state["y"] = x, y
            time.sleep(interval)
    finally:
        with _lock:
            if stopped or event.is_set():
                _state["status"] = "idle"
            else:
                _state["status"] = "finished"


def _angle_of(dx, dy):
    if dx > 0:
        return 0
    if dx < 0:
        return 180
    if dy > 0:
        return 90
    return 270


def stop():
    global _worker
    _stop_event.set()
    worker = _worker
    if worker is not None and worker.is_alive():
        worker.join(timeout=1.0)
    _worker = None
    with _lock:
        if _state["status"] == "moving":
            _state["status"] = "idle"


def reset():
    stop()
    with _lock:
        _state.update(x=0, y=0, angle=0, status="idle")


def _self_test():
    obstacles = [[5, 0], [0, 5], [10, 10]]
    set_map(obstacles, MAP_SIZE)
    reset()

    print("=" * 62)
    print("robot_sim 自测：地图 20×20，障碍物 %s" % obstacles)
    print("=" * 62)

    ok = True

    def check(name, cond, detail):
        nonlocal ok
        ok = ok and cond
        print("[%s] %s  %s" % ("PASS" if cond else "FAIL", name, detail))

    reset()
    check("初始位置", get_state()["x"] == 0 and get_state()["y"] == 0, get_state())

    r = move("right")
    check("正常右移", r["ok"] and get_state()["x"] == 1, get_state())
    check("朝向随移动更新", get_state()["angle"] == 0, get_state())

    reset()
    r = move("up")
    check("越界被拦截", (not r["ok"]) and get_state()["y"] == 0, r)

    reset()
    for _ in range(4):
        move("down")
    r = move("down")
    check("障碍物被拦截", (not r["ok"]) and get_state()["y"] == 4, r)

    reset()
    follow([[0, 0], [1, 0], [2, 0], [2, 1]], 4)
    deadline = time.time() + 3
    while time.time() < deadline:
        if get_state()["status"] != "moving":
            break
        time.sleep(0.02)
    st = get_state()
    check("follow 到达终点", st["x"] == 2 and st["y"] == 1 and st["status"] == "finished", st)

    reset()
    follow([[i, 0] for i in range(0, 11)], 0.5)
    time.sleep(0.25)
    stop()
    st = get_state()
    check("stop 中断且状态回 idle", st["status"] == "idle" and st["x"] < 10, st)

    print("-" * 62)
    print("自测结论：%s" % ("全部通过" if ok else "存在失败用例"))
    return ok


if __name__ == "__main__":
    import sys

    sys.exit(0 if _self_test() else 1)
