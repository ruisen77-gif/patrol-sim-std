# -*- coding: utf-8 -*-
"""机器人运动仿真模块。

参考实现：正式版本由「嵌入式仿真」角色交付，对外函数名、入参、返回字段不得改动。
"""

import threading
import time

MAP_SIZE = 20
STEP_INTERVAL = 0.2  # 1x 速度下每走一格所需时间（秒）

_lock = threading.RLock()
_stop_flag = threading.Event()
_state = {"x": 0, "y": 0, "angle": 0, "is_moving": False, "task_status": "idle"}

_DELTA = {"up": (0, -1), "down": (0, 1), "left": (-1, 0), "right": (1, 0)}


def _in_map(x, y):
    return 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE


def move_command(direction):
    """单格手动移动。方向非法或会走出地图时不移动。"""
    if direction not in _DELTA:
        return
    dx, dy = _DELTA[direction]
    with _lock:
        nx, ny = _state["x"] + dx, _state["y"] + dy
        if not _in_map(nx, ny):
            return
        _state["x"], _state["y"] = nx, ny


def follow_path(points, speed_multiplier=1):
    """按路径点顺序逐步行进。speed_multiplier 越大走得越快。"""
    try:
        multiplier = float(speed_multiplier)
    except (TypeError, ValueError):
        multiplier = 1.0
    if multiplier <= 0:
        multiplier = 1.0
    interval = STEP_INTERVAL / multiplier

    _stop_flag.clear()
    with _lock:
        _state["is_moving"] = True
        _state["task_status"] = "moving"

    try:
        for point in points:
            if _stop_flag.is_set():
                return
            px, py = int(point[0]), int(point[1])
            if not _in_map(px, py):
                continue
            with _lock:
                _state["x"], _state["y"] = px, py
            time.sleep(interval)
    finally:
        with _lock:
            _state["is_moving"] = False
            _state["task_status"] = "idle"


def stop_move():
    """立即停止一切运动，任务状态回到 idle。"""
    _stop_flag.set()
    with _lock:
        _state["is_moving"] = False
        _state["task_status"] = "idle"


def get_robot_state():
    with _lock:
        return {
            "x": _state["x"],
            "y": _state["y"],
            "angle": 0,
            "is_moving": _state["is_moving"],
            "task_status": _state["task_status"],
        }


def _self_test():
    print("初始状态:", get_robot_state())
    for d in ("right", "right", "down"):
        move_command(d)
    print("手动移动后:", get_robot_state())

    follow_path([[2, 2], [3, 2], [4, 2]], 4)
    print("4x 跟随结束:", get_robot_state())

    threading.Thread(target=follow_path, args=([[5, 5], [6, 5], [7, 5]], 1), daemon=True).start()
    time.sleep(0.3)
    print("1x 跟随中:", get_robot_state())
    stop_move()
    time.sleep(0.1)
    print("停止后:", get_robot_state())


if __name__ == "__main__":
    _self_test()
