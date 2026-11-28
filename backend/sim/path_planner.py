# -*- coding: utf-8 -*-
"""算法模块：BFS 最短路与多巡检点串联。

对外函数（全组契约）：
    bfs(size, obstacles, start, end) -> dict
        {"ok": True,  "path": [[x, y], ...], "msg": ""}
        {"ok": False, "path": [],            "msg": "无法找到路径"}
    patrol(size, obstacles, points)  -> list
        串联后的完整路径；额外携带 unreachable_points 属性，列出被跳过的点位。

坐标系：左上角为 (0, 0)，x 向右、y 向下递增，只允许四方向移动。
本模块只做寻路，不控制机器人、不涉及界面。
"""

from collections import deque

MAP_SIZE = 20
MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1))


class PatrolResult(list):
    """路径列表，附带被跳过的巡检点，便于后端向前端说明原因。"""

    def __init__(self, path, unreachable):
        list.__init__(self, path)
        self.unreachable_points = [list(p) for p in unreachable]


def _normalize(obstacles, size):
    blocked = set()
    for o in obstacles or []:
        x, y = int(o[0]), int(o[1])
        if 0 <= x < size and 0 <= y < size:
            blocked.add((x, y))
    return blocked


def _in_range(point, size):
    return 0 <= point[0] < size and 0 <= point[1] < size


def bfs(size, obstacles, start, end):
    size = int(size)
    blocked = _normalize(obstacles, size)
    start = (int(start[0]), int(start[1]))
    end = (int(end[0]), int(end[1]))

    if not _in_range(start, size) or not _in_range(end, size):
        return {"ok": False, "path": [], "msg": "无法找到路径"}
    if start in blocked or end in blocked:
        return {"ok": False, "path": [], "msg": "无法找到路径"}
    if start == end:
        return {"ok": True, "path": [list(start)], "msg": ""}

    parent = {start: None}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        if (x, y) == end:
            break
        for dx, dy in MOVES:
            nxt = (x + dx, y + dy)
            if nxt in parent or nxt in blocked or not _in_range(nxt, size):
                continue
            parent[nxt] = (x, y)
            queue.append(nxt)

    if end not in parent:
        return {"ok": False, "path": [], "msg": "无法找到路径"}

    path = []
    node = end
    while node is not None:
        path.append(list(node))
        node = parent[node]
    path.reverse()
    return {"ok": True, "path": path, "msg": ""}


def patrol(size, obstacles, points):
    if not points:
        return PatrolResult([], [])

    start = (int(points[0][0]), int(points[0][1]))
    path = [list(start)]
    unreachable = []

    for target in points[1:]:
        target = (int(target[0]), int(target[1]))
        result = bfs(size, obstacles, path[-1], target)
        if not result["ok"]:
            unreachable.append(list(target))
            continue
        path.extend(result["path"][1:])

    return PatrolResult(path, unreachable)


def _self_test():
    obstacles = [[5, 5], [5, 6], [5, 7], [5, 8], [5, 9], [5, 10], [5, 11], [5, 12], [5, 13], [5, 14]]

    print("=" * 62)
    print("path_planner 自测：20×20，x=5 处 y=5..14 为连续障碍墙")
    print("=" * 62)

    ok = True

    def check(name, cond, detail):
        nonlocal ok
        ok = ok and cond
        print("[%s] %s  %s" % ("PASS" if cond else "FAIL", name, detail))

    r = bfs(20, [], [0, 0], [3, 3])
    check("空地寻路", r["ok"] and len(r["path"]) == 7, "步数 %d" % len(r["path"]))

    r = bfs(20, obstacles, [4, 9], [6, 9])
    check("绕墙寻路", r["ok"] and [5, 9] not in r["path"], "步数 %d" % len(r["path"]))

    r = bfs(20, obstacles, [10, 10], [10, 10])
    check("起终点重合", r["ok"] and len(r["path"]) == 1, r)

    cage = [[1, 0], [0, 1], [2, 1], [1, 2]]
    r = bfs(20, cage, [0, 0], [1, 1])
    check("终点被围不可达", (not r["ok"]) and r["msg"] == "无法找到路径", r)

    result = patrol(20, obstacles, [[0, 0], [10, 0], [10, 19], [1, 1]])
    check("三点串联", isinstance(result, list) and result[-1] == [1, 1], "总步数 %d" % len(result))
    check("跳过不可达点", patrol(20, obstacles, [[0, 0], [5, 9]]) .unreachable_points == [[5, 9]],
          patrol(20, obstacles, [[0, 0], [5, 9]]).unreachable_points)

    print("-" * 62)
    print("自测结论：%s" % ("全部通过" if ok else "存在失败用例"))
    return ok


if __name__ == "__main__":
    import sys

    sys.exit(0 if _self_test() else 1)
