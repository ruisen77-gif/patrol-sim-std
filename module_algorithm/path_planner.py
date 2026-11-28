# -*- coding: utf-8 -*-
"""路径规划与巡检调度模块。

参考实现：正式版本由「算法」角色交付，对外函数名、入参、返回字段不得改动。

约定（见 docs/api_spec.md 附录）：
patrol_task 以 patrol_points[0] 作为起点，调用方负责把机器人当前位置放在首位。
"""

from collections import deque

MAP_SIZE = 20
_NEIGHBORS = ((0, -1), (0, 1), (-1, 0), (1, 0))


def bfs_search(map_size, obstacles, start, end):
    """宽度优先搜索最短路径，返回 {"ok": bool, "path": [[x,y],...], "msg": str}。"""
    try:
        size = int(map_size)
    except (TypeError, ValueError):
        size = MAP_SIZE

    blocked = set()
    for item in obstacles or []:
        try:
            blocked.add((int(item[0]), int(item[1])))
        except (TypeError, ValueError, IndexError):
            continue

    try:
        sx, sy = int(start[0]), int(start[1])
        ex, ey = int(end[0]), int(end[1])
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "path": [], "msg": "坐标格式错误"}

    if not (0 <= ex < size and 0 <= ey < size):
        return {"ok": False, "path": [], "msg": "终点越界"}
    if (sx, sy) == (ex, ey):
        return {"ok": True, "path": [[sx, sy]], "msg": "起点即终点"}
    if (ex, ey) in blocked:
        return {"ok": False, "path": [], "msg": "终点为障碍物"}

    prev = {(sx, sy): None}
    queue = deque([(sx, sy)])
    while queue:
        cx, cy = queue.popleft()
        for dx, dy in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < size and 0 <= ny < size):
                continue
            if (nx, ny) in blocked or (nx, ny) in prev:
                continue
            prev[(nx, ny)] = (cx, cy)
            if (nx, ny) == (ex, ey):
                path = [(ex, ey)]
                cur = (ex, ey)
                while prev[cur] is not None:
                    cur = prev[cur]
                    path.append(cur)
                path.reverse()
                return {"ok": True, "path": [list(p) for p in path], "msg": "ok"}
            queue.append((nx, ny))

    return {"ok": False, "path": [], "msg": "无法找到路径"}


def patrol_task(map_size, obstacles, patrol_points):
    """依次连接巡检点位，返回拼接完成的完整路径点数组；不可达点位跳过。"""
    points = []
    for item in patrol_points or []:
        try:
            points.append([int(item[0]), int(item[1])])
        except (TypeError, ValueError, IndexError):
            continue

    if not points:
        return []

    full_path = [list(points[0])]
    current = points[0]
    for target in points[1:]:
        result = bfs_search(map_size, obstacles, current, target)
        if not result["ok"]:
            continue
        full_path.extend(result["path"][1:])
        current = target
    return full_path


def _self_test():
    obstacles = [[5, 5], [6, 5], [7, 5], [5, 6], [5, 7]]
    print("正常寻路:", bfs_search(20, obstacles, [0, 0], [8, 8]))
    print("终点被包围:", bfs_search(20, [[4, 0], [2, 0], [3, 1], [3, -1]], [0, 0], [3, 0]))
    print("多点调度:", patrol_task(20, obstacles, [[0, 0], [8, 8], [3, 3], [18, 18]]))


if __name__ == "__main__":
    _self_test()
