# -*- coding: utf-8 -*-
"""Flask 网关：对外暴露六个接口，对内只做编排。

接口：
    GET  /api/map      地图尺寸、障碍物、起点与巡检点上限
    GET  /api/robot    机器人状态 x / y / angle / status
    POST /api/move     手动移动，失败返回 409 与原因
    POST /api/patrol   启动巡检，返回完整路径
    POST /api/stop     停止巡检
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


class PatrolTracker(object):
    """记录本次巡检计划，按机器人位置推算进度。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.plan = []
        self.points = []
        self.cursor = 0
        self.speed = 1.0

    def start(self, plan, points, speed):
        with self.lock:
            self.plan = [list(p) for p in plan]
            self.points = [list(p) for p in points]
            self.cursor = 0
            self.speed = speed

    def clear(self):
        with self.lock:
            self.plan = []
            self.points = []
            self.cursor = 0

    def sync(self, x, y):
        with self.lock:
            pos = [int(x), int(y)]
            for i in range(self.cursor, len(self.plan)):
                if self.plan[i] == pos:
                    self.cursor = i
                    break

    @staticmethod
    def _last_index(plan, point):
        for i in range(len(plan) - 1, -1, -1):
            if plan[i] == point:
                return i
        return None

    def snapshot(self):
        with self.lock:
            plan = [list(p) for p in self.plan]
            total = len(self.points)
            finished = 0
            target = None
            for point in self.points:
                index = self._last_index(plan, point)
                if index is None:
                    continue
                if index <= self.cursor:
                    finished += 1
                elif target is None:
                    target = list(point)
            return {
                "plan": plan,
                "path_index": self.cursor,
                "total": total,
                "finished": finished,
                "remain": max(0, len(plan) - 1 - self.cursor),
                "target": target,
                "speed": self.speed,
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

        clean = []
        try:
            for item in points:
                x, y = int(item[0]), int(item[1])
                if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
                    return jsonify({"ok": False, "msg": "巡检点 (%d, %d) 越界" % (x, y)}), 409
                if (x, y) in _blocked_set():
                    return jsonify({"ok": False, "msg": "巡检点 (%d, %d) 位于障碍物上" % (x, y)}), 409
                if [x, y] not in clean:
                    clean.append([x, y])
        except (TypeError, ValueError, IndexError):
            return jsonify({"ok": False, "msg": "巡检点格式应为 [[x, y], ...]"}), 409

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

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        robot_sim.stop()
        return jsonify({"ok": True, "status": robot_sim.get_state()["status"]})

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
        }
        data["target"] = snap["target"]
        data["status"] = state["status"]
        return jsonify(data)

    return app


app = create_app()


if __name__ == "__main__":
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5050"))
    app.run(host=host, port=port, threaded=True)
