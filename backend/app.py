# -*- coding: utf-8 -*-
"""Flask 网关：对外暴露九个接口，对内只做编排。

接口：
    GET  /api/map      地图尺寸、障碍物、起点与巡检点上限
    GET  /api/robot    机器人状态 x / y / angle / status
    POST /api/move     手动移动，失败返回 409 与原因
    GET  /api/points   已登记的巡检点
    POST /api/points   登记 / 覆盖巡检点（只记点，不启动移动）
    POST /api/patrol   启动巡检，返回完整路径
    POST /api/speed    运行中调整速度（从当前格按新速度续走）
    POST /api/stop     停止巡检
    POST /api/reset    复位：机器人回起点，清空巡检点与进度
    GET  /api/sensor   传感器读数 + 巡检进度
"""

import os
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(BACKEND_DIR)
FRONTEND_DIR = os.path.join(ROOT, "frontend")
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

from sim import path_planner, robot_sim, sensor_sim  # noqa: E402

MAP_SIZE = 20
MAX_PATROL_POINTS = 6
START = [0, 0]


def _build_obstacles():
    obstacles = [[6, y] for y in range(2, 9)]
    obstacles += [[x, 10] for x in range(10, 15)]
    obstacles += [[16, 4], [16, 5], [16, 6]]
    return obstacles


OBSTACLES = _build_obstacles()
robot_sim.set_map(OBSTACLES, MAP_SIZE)


def _blocked_set():
    return {(o[0], o[1]) for o in OBSTACLES}


def _clean_points(points):
    """校验并去重巡检点，返回 (clean, err)；err 为 None 表示通过。"""
    clean = []
    try:
        for item in points:
            x, y = int(item[0]), int(item[1])
            if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
                return None, "巡检点 (%d, %d) 越界" % (x, y)
            if (x, y) in _blocked_set():
                return None, "巡检点 (%d, %d) 位于障碍物上" % (x, y)
            if [x, y] not in clean:
                clean.append([x, y])
    except (TypeError, ValueError, IndexError):
        return None, "巡检点格式应为 [[x, y], ...]"
    return clean, None


class PatrolTracker(object):
    """记录巡检点与计划路径，按机器人实际到访推算进度。

    进度口径与移动方式无关：
        - 巡检点在 /api/points 或 /api/patrol 时登记；
        - 由后台采样线程（0.1 秒）与 /api/move 调用 sync，记录到访；
        - finished 只统计“按登记顺序到达”的点位：乱序先到的点只记 visited 供提示，
          不计进度，且之后不会被追认，必须重新按顺序走到才计数。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.plan = []
        self.points = []
        self.cursor = 0
        self.counted = []     # 已按顺序计入的点位序列，长度即进度
        self.speed = 1.0
        self.visited = set()

    def start(self, plan, points, speed):
        with self.lock:
            self.plan = [list(p) for p in plan]
            self.points = [list(p) for p in points]
            self.cursor = 0
            self.counted = []
            self.speed = speed
            self._prune_visited()

    def set_points(self, points):
        """登记巡检点；按新列表重新对齐已计入的进度。

        删点、改顺序都会让旧的计数错位，这里按“新列表前缀与旧计入序列逐项一致”
        来重建 counted：在末尾追加点位不丢进度，删掉已计入的点则从该处起清零。
        """
        with self.lock:
            old = self.counted
            self.points = [list(p) for p in points]
            keep = []
            for i, p in enumerate(self.points):
                if i < len(old) and int(old[i][0]) == int(p[0]) and int(old[i][1]) == int(p[1]):
                    keep.append([int(p[0]), int(p[1])])
                else:
                    break
            self.counted = keep
            self._prune_visited()

    def set_speed(self, speed):
        """记录当前巡检速度（不改已算好的路径与进度）。"""
        with self.lock:
            self.speed = float(speed)

    def _prune_visited(self):
        """丢弃已不在巡检点集合中的到访记录（调用方须持有锁）。"""
        keep = {(int(p[0]), int(p[1])) for p in self.points}
        self.visited &= keep

    def clear(self):
        with self.lock:
            self.plan = []
            self.points = []
            self.cursor = 0
            self.counted = []
            self.visited = set()

    def _mark(self, x, y):
        """把一个点位记为“已按顺序到达”（调用方须持有锁）。"""
        self.counted.append([int(x), int(y)])
        self.visited.add((int(x), int(y)))

    def sync(self, x, y, moving=False):
        """用机器人当前坐标记录到访，并按严格顺序推进进度。

        只有坐标等于“当前该去的点位” points[len(counted)] 时才前进一步；
        乱序到达的点只记入 visited（供前端标橙提示），不计进度，
        之后即使前面点位补齐也不会被追认，必须重新按顺序走到才计数。

        moving：机器人是否正处于自动巡检行进中。只有行进中才启用
        路径游标推断（补偿采样漏格），避免自动巡检结束后手动移动时
        旧 plan 把进度“跳着算”。
        """
        with self.lock:
            pos = [int(x), int(y)]
            if pos in self.points:
                self.visited.add((pos[0], pos[1]))
            if len(self.counted) < len(self.points):
                cur = self.points[len(self.counted)]
                if int(cur[0]) == pos[0] and int(cur[1]) == pos[1]:
                    self._mark(pos[0], pos[1])
            for i in range(self.cursor, len(self.plan)):
                if self.plan[i] == pos:
                    self.cursor = i
                    break
            at_end = bool(self.plan) and self.cursor >= len(self.plan) - 1
            if moving or at_end:
                self._catch_up_by_cursor()

    def _catch_up_by_cursor(self):
        """自动巡检的漏采样补偿（调用方须持有锁）。

        采样可能漏掉机器人经过的点位格，因此自动模式下不只看当前坐标，
        而是看路径游标：游标已越过某点位的路径下标，说明这条路径确实走过它，
        按登记顺序依次追认即可。手动模式无 plan，仍只认当前坐标命中。
        """
        if not self.plan:
            return
        while len(self.counted) < len(self.points):
            px = int(self.points[len(self.counted)][0])
            py = int(self.points[len(self.counted)][1])
            idx = None
            for i, cell in enumerate(self.plan):
                if int(cell[0]) == px and int(cell[1]) == py:
                    idx = i
                    break
            if idx is None or idx > self.cursor:
                return
            self._mark(px, py)

    def snapshot(self):
        with self.lock:
            plan = [list(p) for p in self.plan]
            points = [list(p) for p in self.points]
            in_set = {(int(p[0]), int(p[1])) for p in self.points}
            reached = sorted(self.visited & in_set)

            # 严格顺序：finished 就是已按顺序到达的点位数，乱序到访不追认
            finished = len(self.counted)

            counted = {(int(p[0]), int(p[1])) for p in self.counted}
            out_of_order = [list(p) for p in reached if (p[0], p[1]) not in counted]
            target = list(points[finished]) if finished < len(points) else None

            if not points:
                mode = "idle"
            elif plan:
                mode = "auto"
            else:
                mode = "manual"
            return {
                "plan": plan,
                "points": points,
                "path_index": self.cursor,
                "total": len(points),
                "finished": finished,
                "remain": max(0, len(plan) - 1 - self.cursor) if (plan and points) else 0,
                "target": target,
                "speed": self.speed,
                "mode": mode,
                "visited": [list(p) for p in reached],
                "out_of_order": out_of_order,
            }


def create_app():
    app = Flask(__name__, static_folder=FRONTEND_DIR, static_url_path="")
    app.config["ENGINE"] = PatrolTracker()
    tracker = app.config["ENGINE"]

    # 后台采样线程：0.1 秒一次把机器人坐标喂给进度口径，
    # 让进度推进与“客户端是否在轮询”解耦，GET 接口回归纯读。
    _watcher = {"thread": None}

    def _watch_loop():
        while True:
            try:
                state = robot_sim.get_state()
                tracker.sync(state["x"], state["y"], state.get("status") == "moving")
            except Exception:
                pass
            time.sleep(0.1)

    def _ensure_watcher():
        t = _watcher["thread"]
        if t is None or not t.is_alive():
            t = threading.Thread(target=_watch_loop, name="patrol-watcher", daemon=True)
            _watcher["thread"] = t
            t.start()

    @app.after_request
    def add_cors(response):
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.route("/")
    def index():
        return send_from_directory(FRONTEND_DIR, "index.html")

    @app.route("/api/map", methods=["GET"])
    def api_map():
        return jsonify({
            "size": MAP_SIZE,
            "obstacles": OBSTACLES,
            "start": START,
            "max_patrol_points": MAX_PATROL_POINTS,
        })

    @app.route("/api/robot", methods=["GET"])
    def api_robot():
        return jsonify(robot_sim.get_state())

    @app.route("/api/move", methods=["POST"])
    def api_move():
        data = request.get_json(silent=True) or {}
        result = robot_sim.move(str(data.get("direction", "")))
        # 手动每走一格都记录到访，不必等后台采样线程（最坏 0.1 秒）才反映
        state = robot_sim.get_state()
        tracker.sync(state["x"], state["y"], state.get("status") == "moving")
        if not result.get("ok"):
            # 对外不暴露内部方法名
            result["msg"] = str(result.get("msg", "")).replace(
                "请先调用 stop() 停止任务", "请先停止任务")
        code = 200 if result.get("ok") else 409
        return jsonify(result), code

    @app.route("/api/patrol", methods=["POST"])
    def api_patrol():
        data = request.get_json(silent=True) or {}
        points = data.get("points") or []
        speed = data.get("speed", 1)
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "msg": "speed 应为数字"}), 409
        if not any(abs(speed - s) < 1e-9 for s in robot_sim.ALLOWED_SPEEDS):
            return jsonify({"ok": False, "msg": "速度只支持 0.5 / 1 / 2 / 4"}), 409

        clean, err = _clean_points(points)
        if err:
            return jsonify({"ok": False, "msg": err}), 409

        if not clean:
            return jsonify({"ok": False, "msg": "请先在地图上点击设置巡检点"}), 409
        if len(clean) > MAX_PATROL_POINTS:
            return jsonify({"ok": False, "msg": "巡检点最多 %d 个" % MAX_PATROL_POINTS}), 409

        state = robot_sim.get_state()
        route = [[state["x"], state["y"]]] + clean
        raw = path_planner.patrol(MAP_SIZE, OBSTACLES, route)
        unreachable = getattr(raw, "unreachable_points", [])
        plan = [[int(p[0]), int(p[1])] for p in raw]

        if len(plan) < 2:
            return jsonify({"ok": False, "msg": "目标点不可达"}), 409

        # 只把路径真会经过的点位登记进进度口径，避免不可达点让 total 永远达不到
        cells = {(int(c[0]), int(c[1])) for c in plan}
        counted_points = [p for p in clean if (int(p[0]), int(p[1])) in cells]
        for p in clean:
            if (int(p[0]), int(p[1])) not in cells and p not in unreachable:
                unreachable.append(p)

        tracker.start(plan, counted_points, speed)
        robot_sim.follow(plan, speed)
        _ensure_watcher()

        msg = ""
        if unreachable:
            msg = "部分点位不可达，已跳过：%s" % unreachable
        return jsonify({"ok": True, "plan": plan, "unreachable": unreachable,
                        "points": counted_points, "msg": msg})

    @app.route("/api/speed", methods=["POST"])
    def api_speed():
        """运行中调整巡检速度：重算剩余路径，从当前格按新速度续走，不动已算好的进度。"""
        data = request.get_json(silent=True) or {}
        try:
            speed = float(data.get("speed", 1))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "msg": "speed 应为数字"}), 409
        if not any(abs(speed - s) < 1e-9 for s in robot_sim.ALLOWED_SPEEDS):
            return jsonify({"ok": False, "msg": "速度只支持 0.5 / 1 / 2 / 4"}), 409

        state = robot_sim.get_state()
        snap = tracker.snapshot()
        tracker.set_speed(speed)

        if state["status"] != "moving" or not snap["plan"]:
            return jsonify({"ok": True, "applied": False, "speed": speed,
                            "msg": "当前没有进行中的巡检，速度已记录，下次启动生效"})

        plan = [list(p) for p in snap["plan"]]
        pos = [int(state["x"]), int(state["y"])]
        k = None
        for i in range(snap["path_index"], len(plan)):
            if plan[i] == pos:
                k = i
                break
        if k is None:
            for i, cell in enumerate(plan):
                if cell == pos:
                    k = i
                    break
        if k is None:
            return jsonify({"ok": False, "msg": "当前坐标不在巡检路径上，无法调整速度"}), 409

        robot_sim.follow(plan[k:], speed)
        return jsonify({"ok": True, "applied": True, "speed": speed})

    @app.route("/api/points", methods=["GET", "POST"])
    def api_points():
        """GET 返回已登记的巡检点；POST 覆盖登记（只记点，不启动移动）。"""
        if request.method == "GET":
            return jsonify({"ok": True, "points": tracker.snapshot()["points"]})

        data = request.get_json(silent=True) or {}
        clean, err = _clean_points(data.get("points") or [])
        if err:
            return jsonify({"ok": False, "msg": err}), 409
        if len(clean) > MAX_PATROL_POINTS:
            return jsonify({"ok": False, "msg": "巡检点最多 %d 个" % MAX_PATROL_POINTS}), 409

        # 巡检中改点位会让已算好的 plan 与进度口径脱节，直接拒绝
        state = robot_sim.get_state()
        snap = tracker.snapshot()
        if state["status"] == "moving" and snap["plan"]:
            return jsonify({"ok": False, "msg": "巡检进行中，请先停止巡检再修改巡检点"}), 409

        tracker.set_points(clean)
        return jsonify({"ok": True, "points": clean})

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        robot_sim.stop()
        return jsonify({"ok": True, "status": robot_sim.get_state()["status"]})

    @app.route("/api/reset", methods=["POST"])
    def api_reset():
        """复位到初始状态：停止行进、机器人回起点、清空巡检点与进度。"""
        robot_sim.stop()
        robot_sim.set_start(START[0], START[1], 0)
        tracker.clear()
        state = robot_sim.get_state()
        return jsonify({"ok": True, "status": state["status"],
                        "robot": state, "points": []})

    @app.route("/api/sensor", methods=["GET"])
    def api_sensor():
        # 纯读：坐标由后台采样线程喂给 tracker，GET 不再改状态
        state = robot_sim.get_state()
        data = sensor_sim.detect(state, OBSTACLES, MAP_SIZE)
        snap = tracker.snapshot()
        data["progress"] = {
            "finished": snap["finished"],
            "total": snap["total"],
            "remain": snap["remain"],
            "mode": snap["mode"],
        }
        data["visited"] = snap["visited"]
        data["out_of_order"] = snap["out_of_order"]
        data["target"] = snap["target"]
        data["status"] = state["status"]
        return jsonify(data)

    return app


app = create_app()


if __name__ == "__main__":
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5050"))
    app.run(host=host, port=port, threaded=True)
