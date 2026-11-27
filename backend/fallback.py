# -*- coding: utf-8 -*-
"""三个业务模块缺失时的最小兜底实现，保证基础版可单独运行。"""

import threading
import time

MAP_SIZE = 20


class MockRobot(object):
    _DELTA = {"up": (0, -1), "down": (0, 1), "left": (-1, 0), "right": (1, 0)}

    def __init__(self):
        self._lock = threading.RLock()
        self._state = {"x": 0, "y": 0, "angle": 0, "is_moving": False, "task_status": "idle"}
        self._stop = threading.Event()

    def move_command(self, direction):
        if direction not in self._DELTA:
            return
        dx, dy = self._DELTA[direction]
        with self._lock:
            nx, ny = self._state["x"] + dx, self._state["y"] + dy
            if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                self._state["x"], self._state["y"] = nx, ny

    def follow_path(self, points, speed_multiplier=1):
        interval = 0.2 / max(float(speed_multiplier or 1), 0.1)
        self._stop.clear()
        with self._lock:
            self._state["is_moving"] = True
            self._state["task_status"] = "moving"
        try:
            for point in points:
                if self._stop.is_set():
                    return
                with self._lock:
                    self._state["x"], self._state["y"] = int(point[0]), int(point[1])
                time.sleep(interval)
        finally:
            with self._lock:
                self._state["is_moving"] = False
                self._state["task_status"] = "idle"

    def stop_move(self):
        self._stop.set()
        with self._lock:
            self._state["is_moving"] = False
            self._state["task_status"] = "idle"

    def get_robot_state(self):
        with self._lock:
            return dict(self._state)


class MockSensor(object):
    def sensor_detect(self, robot_state, obstacle_list):
        x = int(robot_state.get("x", 0))
        y = int(robot_state.get("y", 0))
        obstacles = {(int(p[0]), int(p[1])) for p in (obstacle_list or [])}
        collision = (x, y) in obstacles
        out_bound = x < 0 or x >= MAP_SIZE or y < 0 or y >= MAP_SIZE
        return {
            "sensor_distance": [0, 0, 0, 0],
            "has_collision": collision,
            "is_out_bound": out_bound,
            "task_abnormal": collision or out_bound,
        }


class MockPlanner(object):
    def bfs_search(self, map_size, obstacles, start, end):
        from collections import deque

        blocked = {(int(p[0]), int(p[1])) for p in (obstacles or [])}
        s = (int(start[0]), int(start[1]))
        e = (int(end[0]), int(end[1]))
        if e in blocked:
            return {"ok": False, "path": [], "msg": "终点为障碍物"}
        prev = {s: None}
        queue = deque([s])
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) == e:
                path, cur = [], (cx, cy)
                while cur is not None:
                    path.append(list(cur))
                    cur = prev[cur]
                return {"ok": True, "path": path[::-1], "msg": "ok"}
            for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < map_size and 0 <= ny < map_size and (nx, ny) not in blocked and (nx, ny) not in prev:
                    prev[(nx, ny)] = (cx, cy)
                    queue.append((nx, ny))
        return {"ok": False, "path": [], "msg": "无法找到路径"}

    def patrol_task(self, map_size, obstacles, patrol_points):
        points = [[int(p[0]), int(p[1])] for p in (patrol_points or [])]
        if not points:
            return []
        full = [points[0]]
        for i in range(1, len(points)):
            res = self.bfs_search(map_size, obstacles, points[i - 1], points[i])
            if res["ok"]:
                full.extend(res["path"][1:])
        return full
