# -*- coding: utf-8 -*-
"""Flask 网关：对外暴露八个接口，对内只做编排。

接口：
    GET  /api/map      地图尺寸、障碍物、起点与巡检点上限
    GET  /api/robot    机器人状态 x / y / angle / status
    POST /api/move     手动移动，失败返回 409 与原因
    GET  /api/points   已登记的巡检点
    POST /api/points   登记 / 覆盖巡检点（只记点，不启动移动）
    POST /api/patrol   启动巡检，返回完整路径
    POST /api/stop     停止巡检
    POST /api/reset    复位：机器人回起点，清空巡检点与进度
    GET  /api/sensor   传感器读数 + 巡检进度
"""

import os
import sys
import threading

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
        - 每次 /api/robot、/api/sensor 读取坐标时，若该格属于巡检点则记入 visited；
        - finished = 已到访的巡检点数，自动巡检与手动操作共用同一套记录。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.plan = []
        self.points = []
        self.cursor = 0
        self.speed = 1.0
        self.visited = set()

    def start(self, plan, points, speed):
        with self.lock:
            self.plan = [list(p) for p in plan]
            self.points = [list(p) for p in points]
            self.cursor = 0
            self.speed = speed
            self._prune_visited()

    def set_points(self, points):
        """只登记巡检点、不启动移动；仍存在的点位保留其“已到访”记录。"""
        with self.lock:
            self.points = [list(p) for p in points]
            self._prune_visited()

    def _prune_visited(self):
        """丢弃已不在巡检点集合中的到访记录（调用方须持有锁）。"""
        keep = {(int(p[0]), int(p[1])) for p in self.points}
        self.visited &= keep

    def clear(self):
        with self.lock:
            self.plan = []
            self.points = []
            self.cursor = 0
            self.visited = set()

    def sync(self, x, y):
        """用机器人当前坐标记录到访，并推进自动巡检路径游标。"""
        with self.lock:
            pos = [int(x), int(y)]
            if pos in self.points:
                self.visited.add((pos[0], pos[1]))
            for i in range(self.cursor, len(self.plan)):
                if self.plan[i] == pos:
                    self.cursor = i
                    break

    def snapshot(self):
        with self.lock:
            plan = [list(p) for p in self.plan]
            points = [list(p) for p in self.points]
            in_set = {(int(p[0]), int(p[1])) for p in self.points}
            reached = sorted(self.visited & in_set)
            target = None
            for point in self.points:
                if (int(point[0]), int(point[1])) not in self.visited:
                    target = list(point)
                    break
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
                "finished": len(reached),
                "remain": max(0, len(plan) - 1 - self.cursor) if (plan and points) else 0,
                "target": target,
                "speed": self.speed,
                "mode": mode,
                "visited": [list(p) for p in reached],
            }


def create_app():
    app = Flask(__name__, static_folder=FRONTEND_DIR, static_url_path="")
    app.config["ENGINE"] = PatrolTracker()
    tracker = app.config["ENGINE"]

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
        state = robot_sim.get_state()
        tracker.sync(state["x"], state["y"])
        return jsonify(state)

    @app.route("/api/move", methods=["POST"])
    def api_move():
        data = request.get_json(silent=True) or {}
        result = robot_sim.move(str(data.get("direction", "")))
        # 手动每走一格都记录到访，避免前端轮询间隔漏掉中间格
        state = robot_sim.get_state()
        tracker.sync(state["x"], state["y"])
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
            speed = 1.0
        if speed <= 0:
            speed = 1.0

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

        tracker.start(plan, clean, speed)
        robot_sim.follow(plan, speed)

        msg = ""
        if unreachable:
            msg = "部分点位不可达，已跳过：%s" % unreachable
        return jsonify({"ok": True, "plan": plan, "unreachable": unreachable, "msg": msg})

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
        state = robot_sim.get_state()
        tracker.sync(state["x"], state["y"])
        data = sensor_sim.detect(state, OBSTACLES, MAP_SIZE)
        snap = tracker.snapshot()
        data["progress"] = {
            "finished": snap["finished"],
            "total": snap["total"],
            "remain": snap["remain"],
            "mode": snap["mode"],
        }
        data["visited"] = snap["visited"]
        data["target"] = snap["target"]
        data["status"] = state["status"]
        return jsonify(data)

    return app


app = create_app()


if __name__ == "__main__":
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5050"))
    app.run(host=host, port=port, threaded=True)
