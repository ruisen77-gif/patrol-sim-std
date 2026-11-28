# -*- coding: utf-8 -*-
"""
sim/path_planner.py —— 智能巡检机器人二维仿真系统 · 算法模块（路径规划）

【模块职责】
    纯计算模块：只负责在二维网格上规划路径，不控制机器人、
    不访问网络、不写界面。由 backend/app.py 调用。

【数据约定】（与 docs/03_项目架构与接口.md 保持一致）
    * 地图为 size×size 网格（本项目 20×20），坐标原点在左上角 (0,0)，
      x 向右递增，y 向下递增，单位为“格”，取值 0 ~ size-1 的整数；
    * 障碍物用二维整数坐标列表表示，如 [[5,5],[5,6],[12,3]]，
      落在这些格子上的位置不可通行；
    * 机器人只能上下左右四方向移动，不能斜穿；
    * 路径统一表示为 [[x,y], ...] 的点位列表，包含起点与终点。

【函数契约】（已写入 docs/03，改动须全员同意）
    bfs(size, obstacles, start, end) -> dict
        成功：{"ok": True,  "path": [[x,y], ...], "msg": ""}
        失败：{"ok": False, "path": [],           "msg": "无法找到路径"}
        起点与终点重合时返回只含一个点的路径。

    patrol(size, obstacles, points) -> dict
        {"ok": bool,
         "path": [[x,y], ...],                # 串联后的完整巡检路径
         "unreachable_points": [[x,y], ...],  # 被跳过的不可达巡检点
         "msg": str}                          # 说明文字，无异常时为空串
        按顺序对相邻两个巡检点调用 bfs，各段路径去掉首点后依次拼接；
        某段失败则跳过该巡检点并记入 unreachable_points，msg 中说明。

【供全栈同学（app.py）接入的写法】
    result = path_planner.patrol(size, obstacles, points)
    if not result["ok"]:
        return {"ok": False, "msg": "目标点不可达"}, 409
    robot_sim.follow(result["path"], speed)
    return {"ok": True, "plan": result["path"]}
"""

from collections import deque

# 四方向移动：右、下、左、上（保证 BFS 结果确定、可复现）
_DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def _in_bounds(x, y, size):
    """判断坐标 (x, y) 是否在地图 0 ~ size-1 范围内。"""
    return 0 <= x < size and 0 <= y < size


def _normalize_obstacles(obstacles, size):
    """把障碍物坐标统一成 (x, y) 元组集合，并过滤掉越界的点。"""
    blocked = set()
    for cell in obstacles or []:
        x, y = int(cell[0]), int(cell[1])
        if _in_bounds(x, y, size):
            blocked.add((x, y))
    return blocked


def bfs(size, obstacles, start, end):
    """
    四方向网格广度优先搜索（BFS），求从 start 到 end 的最短绕障路径。

    参数：
        size      : int，正方形地图边长（本项目为 20）
        obstacles : 障碍物坐标列表，如 [[5,5],[5,6],[12,3]]
        start     : 起点坐标 [x, y]
        end       : 终点坐标 [x, y]

    返回（dict）：
        成功 {"ok": True,  "path": [[x,y], ...], "msg": ""}
        失败 {"ok": False, "path": [], "msg": "无法找到路径"}

    示例：
        >>> bfs(20, [], [0, 0], [2, 0])
        {'ok': True, 'path': [[0, 0], [1, 0], [2, 0]], 'msg': ''}
        >>> bfs(6, [[0, 1], [1, 1], [2, 1], [3, 1], [4, 1], [5, 1]], [0, 0], [0, 2])
        {'ok': False, 'path': [], 'msg': '无法找到路径'}
    """
    sx, sy = int(start[0]), int(start[1])
    ex, ey = int(end[0]), int(end[1])

    # 输入合法性检查：越界坐标直接返回失败并说明原因
    if not _in_bounds(sx, sy, size):
        return {"ok": False, "path": [], "msg": "起点坐标越界"}
    if not _in_bounds(ex, ey, size):
        return {"ok": False, "path": [], "msg": "终点坐标越界"}

    start_cell, end_cell = (sx, sy), (ex, ey)

    # 起点等于终点：按契约返回只含一个点的路径
    if start_cell == end_cell:
        return {"ok": True, "path": [[sx, sy]], "msg": ""}

    blocked = _normalize_obstacles(obstacles, size)

    # 起点或终点本身落在障碍物上，必然无路可走
    if start_cell in blocked or end_cell in blocked:
        return {"ok": False, "path": [], "msg": "无法找到路径"}

    # BFS：queue 存待扩展的格子，parent 记录每个格子从哪里走来
    queue = deque([start_cell])
    parent = {start_cell: None}

    while queue:
        cx, cy = queue.popleft()
        if (cx, cy) == end_cell:
            break
        for dx, dy in _DIRECTIONS:
            nx, ny = cx + dx, cy + dy
            if not _in_bounds(nx, ny, size):
                continue
            if (nx, ny) in blocked:
                continue
            if (nx, ny) in parent:      # 已访问过
                continue
            parent[(nx, ny)] = (cx, cy)
            queue.append((nx, ny))

    # 终点从未被访问到：没有可达路径
    if end_cell not in parent:
        return {"ok": False, "path": [], "msg": "无法找到路径"}

    # 从终点沿 parent 链回溯，再反转得到 起点->终点 的完整路径
    path = []
    cur = end_cell
    while cur is not None:
        path.append([cur[0], cur[1]])
        cur = parent[cur]
    path.reverse()
    return {"ok": True, "path": path, "msg": ""}


def patrol(size, obstacles, points):
    """
    把多个巡检点按顺序串联成一条完整巡检路径（巡检调度）。

    做法：按顺序对相邻两个巡检点调用 bfs，各段路径去掉首点后
    依次拼接；某段失败则跳过该巡检点并记入不可达列表，继续
    尝试下一个点，保证任何单点故障都不会让整个巡检崩溃。

    参数：
        size      : int，地图边长
        obstacles : 障碍物坐标列表
        points    : 巡检点列表，如 [[2,2],[15,3],[18,18]]

    返回（dict）：
        {"ok": bool,
         "path": [[x,y], ...],                # 串联后的完整路径
         "unreachable_points": [[x,y], ...],  # 被跳过的巡检点
         "msg": str}                          # 说明文字，无异常时为空串

    示例（6×6 地图，x=3 一列竖墙，底部留缺口）：
        patrol(6, [[3,0],[3,1],[3,2],[3,3],[3,4]], [[0,0],[5,0]])
        -> {"ok": True, "path": [[0,0],[1,0],[2,0],[2,1],...,[5,0]],
            "unreachable_points": [], "msg": ""}
    """
    result = {"ok": False, "path": [], "unreachable_points": [], "msg": ""}

    if not points:
        result["msg"] = "未提供巡检点"
        return result

    blocked = _normalize_obstacles(obstacles, size)

    # 第一个巡检点是整条巡检的起点，必须本身可达
    ax, ay = int(points[0][0]), int(points[0][1])
    if not _in_bounds(ax, ay, size) or (ax, ay) in blocked:
        result["unreachable_points"] = [[int(p[0]), int(p[1])] for p in points]
        result["msg"] = "起点巡检点不可达，全部巡检点已跳过"
        return result

    full_path = [[ax, ay]]
    skipped = []
    anchor = (ax, ay)

    # 依次串联：锚点（上一个到达的巡检点）-> 下一个巡检点
    for point in points[1:]:
        nx, ny = int(point[0]), int(point[1])
        segment = bfs(size, obstacles, [anchor[0], anchor[1]], [nx, ny])
        if segment["ok"]:
            # 去掉该段首点（与已拼好的路径衔接处重合），再拼接
            full_path.extend(segment["path"][1:])
            anchor = (nx, ny)
        else:
            skipped.append([nx, ny])

    result["path"] = full_path
    result["unreachable_points"] = skipped
    result["ok"] = True
    if skipped:
        result["msg"] = ("已跳过 %d 个不可达巡检点，"
                         "详见 unreachable_points" % len(skipped))
    return result


# ======================================================================
# 自测代码：直接运行 python sim/path_planner.py 即可
# 覆盖：空地寻路、终点被围、起终重合、多点串联（绕障）、跳过不可达点
# ======================================================================

def _check_path(path, size, obstacles, start, end):
    """校验一条路径是否合法：起终点正确、逐格相邻、不越界、不撞障碍。"""
    blocked = _normalize_obstacles(obstacles, size)
    if not path:
        return False, "路径为空"
    if path[0] != [start[0], start[1]]:
        return False, "起点不符"
    if path[-1] != [end[0], end[1]]:
        return False, "终点不符"
    prev = None
    for x, y in path:
        if not _in_bounds(x, y, size):
            return False, "路径越界 (%d,%d)" % (x, y)
        if (x, y) in blocked:
            return False, "路径撞障碍 (%d,%d)" % (x, y)
        if prev is not None and abs(x - prev[0]) + abs(y - prev[1]) != 1:
            return False, "路径不连续 (%d,%d)->(%d,%d)" % (prev[0], prev[1], x, y)
        prev = (x, y)
    return True, ""


def _render_grid(size, obstacles, path, marks=None):
    """在终端用字符画出地图，便于演示与排查。
    marks: {(x, y): 字符}，标记优先级高于路径与空地。"""
    blocked = _normalize_obstacles(obstacles, size)
    path_set = {tuple(p) for p in path}
    marks = marks or {}
    rows = []
    for y in range(size):
        line = []
        for x in range(size):
            if (x, y) in blocked:
                line.append("#")
            elif (x, y) in marks:
                line.append(marks[(x, y)])
            elif (x, y) in path_set:
                line.append("*")
            else:
                line.append(".")
        rows.append("".join(line))
    return "\n".join(rows)


def _self_test():
    print("=" * 60)
    print("sim/path_planner.py 自测：BFS 寻路 + 多点巡检串联")
    print("=" * 60)
    results = []

    def record(name, passed, detail=""):
        results.append(bool(passed))
        tag = "通过" if passed else "失败"
        print("[%s] %s  %s" % (tag, name, detail))

    # ---- 场景一：空地寻路 ----
    print("\n场景一：空地寻路（20×20 无障碍，从 [1,1] 到 [18,18]）")
    r = bfs(20, [], [1, 1], [18, 18])
    good, why = (False, "")
    if r["ok"] and len(r["path"]) == 35:
        good, why = _check_path(r["path"], 20, [], [1, 1], [18, 18])
    record("场景一", good, "path 长度 = %d，msg = \"%s\"" % (len(r["path"]), r["msg"]))
    if r["ok"]:
        print(_render_grid(20, [], r["path"], {(1, 1): "S", (18, 18): "E"}))
    print("  （图例：.空地 #障碍 *路径 S起点 E终点）")

    # ---- 场景二：终点被障碍包围 ----
    print("\n场景二：终点被障碍包围（[5,5] 四面封死，无路可达）")
    r = bfs(20, [[4, 5], [6, 5], [5, 4], [5, 6]], [0, 0], [5, 5])
    ok = (r["ok"] is False) and (r["path"] == []) and (r["msg"] == "无法找到路径")
    record("场景二", ok, "ok=False，path=[]，msg=\"%s\"" % r["msg"])

    # ---- 场景三：起点终点重合 ----
    print("\n场景三：起点与终点重合（[3,3] -> [3,3]）")
    r = bfs(20, [], [3, 3], [3, 3])
    ok = r["ok"] and r["path"] == [[3, 3]] and r["msg"] == ""
    record("场景三", ok, "path = %s，msg = \"%s\"" % (r["path"], r["msg"]))

    # ---- 场景四：三个巡检点串联（绕障） ----
    print("\n场景四：三个巡检点串联（6×6，x=3 竖墙底部留缺口）")
    wall = [[3, y] for y in range(5)]   # x=3 的竖墙，y=0..4，(3,5) 是缺口
    points = [[0, 0], [5, 0], [0, 5]]
    r = patrol(6, wall, points)
    good, why = (False, "")
    if (r["ok"] and r["unreachable_points"] == []
            and r["path"][0] == [0, 0] and r["path"][-1] == [0, 5]
            and [5, 0] in r["path"]):
        good, why = _check_path(r["path"], 6, wall, [0, 0], [0, 5])
    record("场景四", good, "path 长度 = %d，msg = \"%s\"" % (len(r["path"]), r["msg"]))
    if r["ok"]:
        print(_render_grid(6, wall, r["path"], {(0, 0): "1", (5, 0): "2", (0, 5): "3"}))
    print("  （图例：.空地 #障碍 *路径 1/2/3 巡检点）")

    # ---- 场景五（附加）：中间巡检点不可达，跳过并记录 ----
    print("\n场景五（附加）：中间巡检点不可达 -> 跳过并记录")
    wall2 = [[5, y] for y in range(6)]   # x=5 整列封死
    r = patrol(6, wall2, [[0, 0], [5, 5], [0, 5]])
    ok = (r["ok"]
          and r["unreachable_points"] == [[5, 5]]
          and r["path"] == [[0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5]])
    record("场景五", ok, "跳过点 = %s，msg = \"%s\"" % (r["unreachable_points"], r["msg"]))

    print("\n" + "=" * 60)
    passed, total = sum(results), len(results)
    print("自测结果：%d/%d 个场景通过" % (passed, total))
    print("=" * 60)
    return passed == total


if __name__ == "__main__":
    import sys
    sys.exit(0 if _self_test() else 1)


__all__ = ["bfs", "patrol"]
