"""
robot_sim.py —— 智能巡检机器人二维仿真系统 · 嵌入式仿真模块

纯 Python 标准库实现，不依赖任何第三方包，可直接被 Flask 后端 `from sim import robot_sim` 引用。

坐标约定（与 docs/03 一致）：
    - 原点 (0, 0) 在地图左上角，x 向右递增，y 向下递增，单位为“格”；
    - 地图为 size × size 网格，默认 20×20，合法坐标 0 ~ size-1；
    - 障碍物为二维整数坐标列表，如 [[5, 5], [5, 6], [12, 3]]，落在其中的格子不可通行。

朝向约定（angle）：
    0 向右、90 向下、180 向左、270 向上。

对外契约（全组共用，函数名 / 入参 / 返回字段不得改动）：
    move(direction)          -> dict   {"ok": True} 或 {"ok": False, "msg": "原因"}
    get_state()              -> dict   {"x": int, "y": int, "angle": int, "status": str}
    follow(points, speed)    -> None   后台线程逐点行进
    stop()                   -> None   终止行进线程，status 置回 idle
    set_map(obstacles, size) -> None   后端注入地图

附加辅助（可选，不影响上面的契约）：
    set_start(x, y, angle=0) -> None   设置机器人起点与初始朝向
    get_last_report()        -> dict   最近一次巡检的执行报告（到达点数 / 是否中断 / 中断原因）
"""

import threading
import time

# ---------------------------------------------------------------- 常量与内部状态

SIZE_DEFAULT = 20          # 默认网格边长
STEP_SECONDS = 0.4         # 1x 速度下走一格的耗时（秒），0.5x=0.8s / 2x=0.2s / 4x=0.1s
ALLOWED_SPEEDS = (0.5, 1, 2, 4)

# direction -> (dx, dy, 新朝向)
DIRECTIONS = {
    "up":    (0, -1, 270),
    "down":  (0, 1, 90),
    "left":  (-1, 0, 180),
    "right": (1, 0, 0),
}

_lock = threading.RLock()                    # 保护下面的模块级状态
_size = SIZE_DEFAULT
_obstacles = set()                           # 规范化后的障碍物集合 {(x, y), ...}
_x, _y, _angle = 0, 0, 0
_status = "idle"                             # idle / moving / finished
_thread = None                               # 行进线程
_stop_event = threading.Event()

_report = {                                  # 最近一次巡检的执行报告
    "skipped": [],                           # 保留字段（本实现不再跳过，恒为空列表）
    "visited": 0,                            # 实际到达的点位数
    "finished": None,                        # True 整条路线走完 / False 被中断 / None 未开始
    "failed": False,                         # 是否因碰撞/越界中断
    "failed_point": None,                     # 中断发生时所处的目标点位
    "msg": "",                               # 中断原因，供前端弹窗与 /api/sensor 使用
}


# ---------------------------------------------------------------- 内部工具

def _cell_reason(nx, ny):
    """判断某个格子能否进入，返回空字符串表示可通行。"""
    if not (0 <= nx < _size and 0 <= ny < _size):
        return "out_of_bound"
    if (nx, ny) in _obstacles:
        return "blocked"
    return ""


def _do_move(direction):
    """在已持有 _lock 的前提下执行一次单格移动，不改状态机语义。"""
    global _x, _y, _angle, _status

    if not isinstance(direction, str):
        return {"ok": False, "msg": "方向参数无效：只接受 up/down/left/right"}
    key = direction.strip().lower()
    if key not in DIRECTIONS:
        return {"ok": False, "msg": "方向参数无效：只接受 up/down/left/right"}

    dx, dy, new_angle = DIRECTIONS[key]
    nx, ny = _x + dx, _y + dy

    reason = _cell_reason(nx, ny)
    if reason == "out_of_bound":
        return {
            "ok": False,
            "msg": "越界：目标格 (%d, %d) 超出 0-%d 范围，移动被阻止" % (nx, ny, _size - 1),
        }
    if reason == "blocked":
        return {
            "ok": False,
            "msg": "碰撞：目标格 (%d, %d) 为障碍物，移动被阻止" % (nx, ny),
        }

    _x, _y, _angle = nx, ny, new_angle
    if _status == "finished":                # 巡检结束后的手动操作，状态回到 idle
        _status = "idle"
    return {"ok": True}


def _greedy_direction(tx, ty):
    """从当前位置朝目标点选一个主轴方向（点数相邻时即为唯一一步）。"""
    with _lock:
        dx, dy = tx - _x, ty - _y

    if dx == 0 and dy == 0:
        return None
    primary = "right" if dx > 0 else "left"
    if dy != 0 and abs(dy) > abs(dx):
        primary = "down" if dy > 0 else "up"
    return primary


def _normalize_points(points):
    """把外部传入的点位列表规范化为 [(x, y), ...]，非法项直接丢弃。"""
    result = []
    if not isinstance(points, (list, tuple)):
        return result
    for item in points:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue
        try:
            px, py = int(item[0]), int(item[1])
        except (TypeError, ValueError):
            continue
        result.append((px, py))
    return result


def _interval_from_speed(speed):
    """速度系数换算为每格间隔秒数；非法值回落到 1x。"""
    try:
        value = float(speed)
    except (TypeError, ValueError):
        value = 1.0
    if value <= 0 or value > 16:
        value = 1.0
    return STEP_SECONDS / value


def _run_follow(points, interval):
    """行进线程主体：逐点推进，遇障碍/越界立即中断，状态回 idle 并记录失败原因。"""
    global _status, _report

    visited = 0
    failed_point = None
    msg = ""

    try:
        for tx, ty in points:
            if _stop_event.is_set():
                break

            with _lock:
                if (_x, _y) == (tx, ty):
                    visited += 1
                    continue

                direction = _greedy_direction(tx, ty)
                if direction is None:
                    result = {"ok": False, "msg": "点位与当前坐标不相邻，无法单步到达"}
                else:
                    result = _do_move(direction)

            if not result.get("ok"):
                failed_point = [tx, ty]
                msg = result.get("msg", "行进受阻")
                break

            visited += 1
            if _stop_event.wait(interval):   # 等待期间被 stop() 打断则立即退出
                break
    finally:
        stopped = _stop_event.is_set()
        complete = (not stopped and failed_point is None)
        with _lock:
            _status = "finished" if complete else "idle"   # finished 仅代表整条路线走完
            if failed_point is not None:
                msg = "巡检中断于点位 %s：%s" % (failed_point, msg)
            elif stopped:
                msg = "行进被 stop() 终止"
            _report = {
                "skipped": [],
                "visited": visited,
                "finished": (not stopped and failed_point is None),
                "failed": failed_point is not None,
                "failed_point": failed_point,
                "msg": msg,
            }


# ---------------------------------------------------------------- 对外契约

def set_map(obstacles=None, size=SIZE_DEFAULT):
    """后端启动时注入地图；障碍物越界坐标自动丢弃，机器人坐标夹回合法范围。"""
    global _obstacles, _size, _x, _y

    with _lock:
        try:
            new_size = int(size)
        except (TypeError, ValueError):
            new_size = SIZE_DEFAULT
        if new_size <= 0:
            new_size = SIZE_DEFAULT

        clean = set()
        for item in obstacles or []:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                continue
            try:
                ox, oy = int(item[0]), int(item[1])
            except (TypeError, ValueError):
                continue
            if 0 <= ox < new_size and 0 <= oy < new_size:
                clean.add((ox, oy))

        _size = new_size
        _obstacles = clean
        _x = min(max(_x, 0), new_size - 1)
        _y = min(max(_y, 0), new_size - 1)
    return None


def move(direction):
    """手动单格移动：越界或撞障时坐标不变并返回具体原因。"""
    with _lock:
        if _status == "moving":
            return {"ok": False, "msg": "巡检进行中，请先调用 stop() 停止任务再手动控制"}
        return _do_move(direction)


def get_state():
    """返回机器人当前坐标、朝向与运行状态。"""
    with _lock:
        return {"x": _x, "y": _y, "angle": _angle, "status": _status}


def follow(points, speed=1):
    """后台线程按点位序列逐格行进；speed 取 0.5 / 1 / 2 / 4。"""
    global _status, _report, _thread

    points = _normalize_points(points)
    interval = _interval_from_speed(speed)

    if not points:
        with _lock:
            _report = {"skipped": [], "visited": 0, "finished": True,
                       "failed": False, "failed_point": None,
                       "msg": "点位列表为空或格式无效，未执行移动"}
            _status = "finished"
        return None

    # 若上一次巡检仍在跑，先打停并等它退出，避免两条线程同时改坐标
    with _lock:
        old = _thread if (_thread is not None and _thread.is_alive()) else None
        if old is not None:
            _stop_event.set()

    if old is not None:
        old.join(timeout=2.0 + STEP_SECONDS / 0.5)

    with _lock:
        _stop_event.clear()
        _report = {"skipped": [], "visited": 0, "finished": False,
                   "failed": False, "failed_point": None, "msg": ""}
        _status = "moving"
        _thread = threading.Thread(
            target=_run_follow,
            args=(points, interval),
            name="robot-sim-follow",
            daemon=True,
        )
        _thread.start()
    return None


def stop():
    """终止行进线程，并把状态置回 idle。"""
    with _lock:
        thread = _thread
        _stop_event.set()

    if thread is not None and thread.is_alive():
        thread.join(timeout=2.0 + STEP_SECONDS / 0.5)

    with _lock:
        _status = "idle"
    return None


def set_start(x=0, y=0, angle=0):
    """可选辅助：设置机器人起点与初始朝向（越界自动夹取，朝向取最近的合法值）。"""
    global _x, _y, _angle, _status

    with _lock:
        try:
            nx, ny = int(x), int(y)
        except (TypeError, ValueError):
            nx, ny = 0, 0
        _x = min(max(nx, 0), _size - 1)
        _y = min(max(ny, 0), _size - 1)

        try:
            na = int(angle)
        except (TypeError, ValueError):
            na = 0
        _angle = min((0, 90, 180, 270), key=lambda a: min(abs(a - na) % 360, 360 - abs(a - na) % 360))
        _status = "idle"
    return None


def get_last_report():
    """可选辅助：读取最近一次巡检的执行报告，供后端合并进 /api/sensor 的进度与告警字段。"""
    with _lock:
        return {
            "skipped": [list(p) for p in _report.get("skipped", [])],
            "visited": _report.get("visited", 0),
            "finished": _report.get("finished", None),
            "failed": _report.get("failed", False),
            "failed_point": (list(_report["failed_point"])
                             if _report.get("failed_point") else None),
            "msg": _report.get("msg", ""),
        }


# ---------------------------------------------------------------- 自测

if __name__ == "__main__":
    def show(tag):
        state = get_state()
        print("    %-6s x=%2d y=%2d angle=%3d status=%s"
              % (tag, state["x"], state["y"], state["angle"], state["status"]))

    print("=" * 60)
    print("robot_sim 自测开始（1x = %.1f 秒/格）" % STEP_SECONDS)
    print("=" * 60)

    # --- 1. 注入地图 ---------------------------------------------------------
    set_map([[5, 5], [5, 6], [12, 3]], 20)
    print("[1] set_map 注入地图：size=20, obstacles=[[5, 5], [5, 6], [12, 3]]")
    print("    初始状态：", get_state())
    set_map([[5, 5], [30, 2]], 20)
    print("    set_map 丢越界坐标 (30, 2) 后 ->", get_state(), "障碍物仅保留 (5, 5)")

    # --- 2. 连续移动 ---------------------------------------------------------
    set_start(2, 2, 0)
    print("[2] 连续调用 move（起点 2,2 朝向 0）")
    for d in ("right", "right", "down"):
        print("    move('%s') -> %s" % (d, move(d)))
        show("")
    expect = {"x": 4, "y": 3, "angle": 90}
    state = get_state()
    print("    校验：", "通过" if all(state[k] == v for k, v in expect.items()) else "失败", state)

    # --- 3. 撞障碍 -----------------------------------------------------------
    set_start(4, 5, 0)
    result = move("right")           # (5,5) 是障碍物
    print("[3] 撞障碍：机器人在 (4,5)，move('right') ->", result)
    show("")
    print("    校验：", "通过" if (not result["ok"] and (get_state()["x"], get_state()["y"]) == (4, 5)) else "失败")

    # --- 4. 越界 -------------------------------------------------------------
    set_start(0, 0, 270)
    for d in ("up", "left"):
        result = move(d)
        print("[4] 越界：机器人在 (0,0)，move('%s') -> %s" % (d, result))
        show("")
    ok4 = (get_state()["x"], get_state()["y"]) == (0, 0)
    print("    校验：", "通过" if ok4 else "失败")

    # --- 5. 非法方向 ---------------------------------------------------------
    print("[5] 非法方向：move('右上') ->", move("右上"))

    # --- 6. follow：开阔路线应 finished --------------------------------------
    set_start(0, 0, 0)
    print("[6] follow 开阔路线 [[1,0],[2,0],[2,1],[2,2]]，speed=4")
    follow([[1, 0], [2, 0], [2, 1], [2, 2]], 4)
    while get_state()["status"] == "moving":
        time.sleep(0.05)
    show("")
    print("    report =", get_last_report())
    print("    校验：",
          "通过" if (get_state()["status"] == "finished"
                   and get_last_report()["visited"] == 4
                   and get_last_report()["skipped"] == []) else "失败")

    # --- 7. follow：遇障立即中断，状态回 idle 并给出原因 -----------------------
    set_map([[3, 2]], 20)
    set_start(0, 2, 0)
    print("[7] follow [[1,2],[2,2],[3,2],[4,2]]，(3,2) 为障碍，speed=4")
    follow([[1, 2], [2, 2], [3, 2], [4, 2]], 4)
    while get_state()["status"] == "moving":
        time.sleep(0.05)
    show("")
    report = get_last_report()
    print("    report =", report)
    print("    校验：",
          "通过" if (get_state()["status"] == "idle"
                   and report["failed"] is True
                   and report["failed_point"] == [3, 2]
                   and report["finished"] is False
                   and (get_state()["x"], get_state()["y"]) == (2, 2)) else "失败")

    # --- 8. stop：行进中被终止回 idle ---------------------------------------
    set_map([], 20)
    set_start(0, 0, 0)
    print("[8] follow 长路线后 0.5 秒调 stop()，speed=1（0.4 秒/格）")
    follow([[i, 0] for i in range(1, 10)], 1)
    time.sleep(0.5)
    stop()
    show("")
    print("    report =", get_last_report())
    print("    校验：", "通过" if get_state()["status"] == "idle" else "失败")

    # --- 9. follow 后手动移动应恢复 idle ------------------------------------
    set_map([], 20)
    set_start(0, 0, 0)
    follow([[1, 0]], 4)
    while get_state()["status"] == "moving":
        time.sleep(0.05)
    print("[9] 巡检结束后手动 move('down') ->", move("down"))
    show("")
    print("    校验：", "通过" if get_state()["status"] == "idle" and get_state()["y"] == 1 else "失败")

    print("=" * 60)
    print("自测结束")
    print("=" * 60)