# -*- coding: utf-8 -*-
"""Flask 网关：对外提供 HTTP 接口，对内调用仿真、传感器、算法三个模块。"""

import os
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONTEND_DIR = os.path.join(ROOT, "frontend")
for _sub in ("module_embed_sim", "module_ai_sensor", "module_algorithm"):
    _path = os.path.join(ROOT, _sub)
    if _path not in sys.path:
        sys.path.insert(0, _path)

MAP_SIZE = 20
MAX_PATROL_POINTS = 6

try:
    import robot_sim as _robot
except Exception:
    _robot = None

try:
    import sensor_sim as _sensor
except Exception:
    _sensor = None

try:
    import path_planner as _planner
except Exception:
    _planner = None

if _robot is None or _sensor is None or _planner is None:
    from fallback import MockPlanner, MockRobot, MockSensor

    _robot = _robot or MockRobot()
    _sensor = _sensor or MockSensor()
    _planner = _planner or MockPlanner()


def _build_obstacles():
    obstacles = []
    for y in range(2, 15):
        if y != 8:
            obstacles.append([6, y])
    for x in range(10, 19):
        if x != 14:
            obstacles.append([10, x])
    for x in (2, 3, 4):
        for y in (16, 17):
            obstacles.append([x, y])
    obstacles.extend([[15, 12], [16, 12], [15, 13]])
    return obstacles


OBSTACLES = _build_obstacles()
PRESET_POINTS = [[3, 4], [15, 5], [15, 16], [4, 13]]
DIRECTIONS = {"up": (0, -1), "down": (0, 1), "left": (-1, 0), "right": (1, 0)}


class SimEngine(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.speed = 1.0
        self.full_path = []
        self.path_index = 0
        self.trajectory = []
        self.patrol_points = []
        self.patrolling = False
        self.alerts = []
        self.alert_seq = 0
        self._worker = None
        self._stop_event = threading.Event()

    # ---------- 告警 ----------
    def push_alert(self, kind, message):
        with self.lock:
            self.alert_seq += 1
            self.alerts.append({
                "id": self.alert_seq,
                "type": kind,
                "msg": message,
                "ts": round(time.time(), 3),
            })
            if len(self.alerts) > 20:
                self.alerts = self.alerts[-20:]
            return self.alert_seq

    # ---------- 进度 ----------
    def _sync_progress_locked(self):
        state = _robot.get_robot_state()
        pos = [int(state.get("x", 0)), int(state.get("y", 0))]
        if not self.full_path:
            if not self.trajectory or self.trajectory[-1] != pos:
                self.trajectory.append(pos)
            return

        found = None
        for i in range(self.path_index, len(self.full_path)):
            if self.full_path[i] == pos:
                found = i
                break
        if found is None:
            for i in range(len(self.full_path)):
                if self.full_path[i] == pos:
                    found = i
                    break
        if found is not None and found >= self.path_index:
            self.path_index = found
        self.trajectory = [list(p) for p in self.full_path[: self.path_index + 1]]

    def _progress_locked(self):
        total = len(self.patrol_points)
        if not self.full_path:
            return total, 0, 0, None

        finished = 0
        target = None
        for point in self.patrol_points:
            index = None
            for i in range(len(self.full_path) - 1, -1, -1):
                if self.full_path[i] == point:
                    index = i
                    break
            if index is None:
                continue
            if index <= self.path_index:
                finished += 1
            elif target is None:
                target = list(point)
        remain = max(0, len(self.full_path) - 1 - self.path_index)
        return total, finished, remain, target

    # ---------- 手动 ----------
    def move(self, direction):
        if direction not in DIRECTIONS:
            return {"ok": False, "msg": "方向必须是 up/down/left/right"}
        with self.lock:
            if self.patrolling:
                self._stop_patrol_locked()
            state = _robot.get_robot_state()
            dx, dy = DIRECTIONS[direction]
            tx, ty = int(state["x"]) + dx, int(state["y"]) + dy

            if (tx, ty) in {(o[0], o[1]) for o in OBSTACLES}:
                self.push_alert("collision", "前方 (%d, %d) 存在障碍物，无法前进" % (tx, ty))
                return {"ok": False, "alert": "collision", "msg": "碰撞：前方有障碍物"}
            if not (0 <= tx < MAP_SIZE and 0 <= ty < MAP_SIZE):
                self.push_alert("out_bound", "目标坐标 (%d, %d) 越出地图边界" % (tx, ty))
                return {"ok": False, "alert": "out_bound", "msg": "越界：不能走出地图"}

            _robot.move_command(direction)
            self._sync_progress_locked()
            return {"ok": True, "msg": "移动成功"}

    # ---------- 巡检 ----------
    def start_patrol(self, points, speed_multiplier):
        clean = []
        for item in points or []:
            try:
                x, y = int(item[0]), int(item[1])
            except (TypeError, ValueError, IndexError):
                return {"ok": False, "alert": "plan_failed", "msg": "巡检点坐标格式错误"}
            if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
                return {"ok": False, "alert": "plan_failed", "msg": "巡检点 (%d, %d) 越界" % (x, y)}
            if (x, y) in {(o[0], o[1]) for o in OBSTACLES}:
                return {"ok": False, "alert": "plan_failed", "msg": "巡检点 (%d, %d) 位于障碍物上" % (x, y)}
            if [x, y] not in clean:
                clean.append([x, y])
        if not clean:
            return {"ok": False, "alert": "plan_failed", "msg": "请先在地图上添加巡检点"}
        if len(clean) > MAX_PATROL_POINTS:
            return {"ok": False, "alert": "plan_failed", "msg": "巡检点最多 %d 个" % MAX_PATROL_POINTS}

        try:
            speed = float(speed_multiplier)
        except (TypeError, ValueError):
            speed = 1.0
        if speed <= 0:
            speed = 1.0

        with self.lock:
            self._stop_patrol_locked()
            state = _robot.get_robot_state()
            start = [int(state["x"]), int(state["y"])]
            route = [start] + clean
            path = _planner.patrol_task(MAP_SIZE, OBSTACLES, route)
            path = [[int(p[0]), int(p[1])] for p in (path or [])]

            if len(path) < 2:
                self.push_alert("plan_failed", "巡检点均不可达，路径规划失败")
                return {"ok": False, "alert": "plan_failed", "msg": "路径规划失败：没有可达的巡检点"}

            self.full_path = path
            self.path_index = 0
            self.trajectory = [path[0]]
            self.patrol_points = clean
            self.speed = speed
            self.patrolling = True
            self._stop_event = threading.Event()

            self._worker = threading.Thread(
                target=self._patrol_worker,
                args=(path, speed, self._stop_event),
                daemon=True,
            )
            self._worker.start()
        return {"ok": True, "msg": "巡检已启动", "path_length": len(path)}

    def _follow(self, path, speed):
        try:
            _robot.follow_path(path, speed)
        except Exception as exc:  # 模块异常不应拖垮后端
            self.push_alert("plan_failed", "巡检执行异常：%s" % exc)

    def _patrol_worker(self, path, speed, stop_event):
        inner = threading.Thread(target=self._follow, args=(path, speed), daemon=True)
        inner.start()
        try:
            while inner.is_alive() and not stop_event.is_set():
                with self.lock:
                    self._sync_progress_locked()
                self._check_sensor()
                time.sleep(0.02)
        finally:
            if stop_event.is_set():
                _robot.stop_move()
            inner.join(timeout=1.0)
            with self.lock:
                self._sync_progress_locked()
                self.patrolling = False

    def _check_sensor(self):
        state = _robot.get_robot_state()
        result = _sensor.sensor_detect(state, OBSTACLES)
        if result.get("has_collision"):
            with self.lock:
                if not self.alerts or self.alerts[-1]["type"] != "collision":
                    self.push_alert("collision", "传感器检测到碰撞")
        elif result.get("is_out_bound"):
            with self.lock:
                if not self.alerts or self.alerts[-1]["type"] != "out_bound":
                    self.push_alert("out_bound", "传感器检测到越界")

    def _stop_patrol_locked(self):
        if self.patrolling:
            self._stop_event.set()
            _robot.stop_move()
            self.patrolling = False

    def stop(self):
        with self.lock:
            self._stop_patrol_locked()
            _robot.stop_move()
        return {"ok": True, "msg": "已停止"}

    def reset(self):
        with self.lock:
            self._stop_patrol_locked()
            _robot.stop_move()
        time.sleep(0.05)
        try:
            _robot.follow_path([[0, 0]], 4)
        except Exception:
            pass
        with self.lock:
            self.full_path = []
            self.path_index = 0
            self.trajectory = []
            self.patrol_points = []
            self.alerts = []
            self.alert_seq = 0
            self.patrolling = False
        return {"ok": True, "msg": "已复位"}

    # ---------- 状态 ----------
    def status(self):
        with self.lock:
            self._sync_progress_locked()
            state = _robot.get_robot_state()
            sensor = _sensor.sensor_detect(state, OBSTACLES)
            total, finished, remain, target = self._progress_locked()
            return {
                "x": int(state.get("x", 0)),
                "y": int(state.get("y", 0)),
                "angle": 0,
                "is_moving": bool(state.get("is_moving", False)),
                "task_status": state.get("task_status", "idle"),
                "sensor_distance": sensor.get("sensor_distance", [0, 0, 0, 0]),
                "has_collision": bool(sensor.get("has_collision", False)),
                "is_out_bound": bool(sensor.get("is_out_bound", False)),
                "task_abnormal": bool(sensor.get("task_abnormal", False)),
                "patrol_total": total,
                "patrol_finished": finished,
                "remain_steps": remain,
                "current_target": target,
                "full_path": [list(p) for p in self.full_path],
                "path_index": self.path_index,
                "trajectory": [list(p) for p in self.trajectory],
                "patrolling": self.patrolling,
                "speed": self.speed,
                "alerts": list(self.alerts[-8:]),
            }


def create_app():
    app = Flask(__name__, static_folder=None)
    engine = SimEngine()
    app.config["ENGINE"] = engine

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
            "patrol_points": PRESET_POINTS,
            "max_patrol_points": MAX_PATROL_POINTS,
            "start": [0, 0],
        })

    @app.route("/api/move", methods=["POST"])
    def api_move():
        data = request.get_json(silent=True) or {}
        return jsonify(engine.move(str(data.get("direction", ""))))

    @app.route("/api/start_patrol", methods=["POST"])
    def api_start_patrol():
        data = request.get_json(silent=True) or {}
        return jsonify(engine.start_patrol(
            data.get("patrol_points"),
            data.get("speed_multiplier", 1),
        ))

    @app.route("/api/status", methods=["GET"])
    def api_status():
        return jsonify(engine.status())

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        return jsonify(engine.stop())

    @app.route("/api/reset", methods=["POST"])
    def api_reset():
        return jsonify(engine.reset())

    @app.route("/api/health", methods=["GET"])
    def api_health():
        return jsonify({"ok": True})

    return app


app = create_app()


if __name__ == "__main__":
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5050"))
    app.run(host=host, port=port, threaded=True)
