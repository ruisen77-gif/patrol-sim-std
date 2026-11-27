# -*- coding: utf-8 -*-
"""虚拟传感器模块。

参考实现：正式版本由「人工智能」角色交付，对外函数名、入参、返回字段不得改动。
"""

MAP_SIZE = 20


def sensor_detect(robot_state, obstacle_list):
    """接收机器人状态与障碍物列表，输出传感器数据与异常标志。"""
    try:
        x = int(robot_state.get("x", 0))
        y = int(robot_state.get("y", 0))
    except (TypeError, ValueError):
        x, y = 0, 0

    obstacles = set()
    for item in obstacle_list or []:
        try:
            obstacles.add((int(item[0]), int(item[1])))
        except (TypeError, ValueError, IndexError):
            continue

    has_collision = (x, y) in obstacles
    is_out_bound = x < 0 or x >= MAP_SIZE or y < 0 or y >= MAP_SIZE

    return {
        "sensor_distance": [0, 0, 0, 0],
        "has_collision": has_collision,
        "is_out_bound": is_out_bound,
        "task_abnormal": bool(has_collision or is_out_bound),
    }


def _self_test():
    obstacles = [[5, 5], [6, 5], [5, 6]]
    cases = [
        {"x": 3, "y": 3},
        {"x": 5, "y": 5},
        {"x": -1, "y": 4},
        {"x": 20, "y": 0},
    ]
    for state in cases:
        print(state, "->", sensor_detect(state, obstacles))


if __name__ == "__main__":
    _self_test()
