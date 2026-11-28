# -*- coding: utf-8 -*-
"""后端与三个仿真模块的验收测试。"""

import time

import pytest

from app import MAP_SIZE, OBSTACLES, create_app
from sim import path_planner, robot_sim, sensor_sim

BLOCKED = {(o[0], o[1]) for o in OBSTACLES}


@pytest.fixture()
def client():
    robot_sim.set_map(OBSTACLES, MAP_SIZE)
    robot_sim.reset()
    app = create_app()
    app.config["TESTING"] = True
    c = app.test_client()
    yield c
    robot_sim.stop()
    robot_sim.reset()


def wait_finished(client, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        status = client.get("/api/robot").get_json()["status"]
        if status != "moving":
            return status
        time.sleep(0.05)
    return "moving"


# ---------- 模块：机器人 ----------

def test_move_and_orientation():
    robot_sim.set_map(OBSTACLES, MAP_SIZE)
    robot_sim.reset()
    assert robot_sim.move("right")["ok"] is True
    assert robot_sim.move("down")["ok"] is True
    state = robot_sim.get_state()
    assert [state["x"], state["y"]] == [1, 1]
    assert state["angle"] == 90


def test_move_blocked_by_wall():
    robot_sim.set_map([[1, 0]], MAP_SIZE)
    robot_sim.reset()
    result = robot_sim.move("right")
    assert result["ok"] is False and "障碍" in result["msg"]
    assert robot_sim.get_state()["x"] == 0


def test_move_out_of_bound():
    robot_sim.set_map([], MAP_SIZE)
    robot_sim.reset()
    result = robot_sim.move("up")
    assert result["ok"] is False and "越出" in result["msg"]
    assert robot_sim.get_state()["y"] == 0


def test_follow_reaches_end():
    robot_sim.set_map([], MAP_SIZE)
    robot_sim.reset()
    robot_sim.follow([[0, 0], [1, 0], [2, 0], [2, 1]], 4)
    deadline = time.time() + 3
    while time.time() < deadline and robot_sim.get_state()["status"] == "moving":
        time.sleep(0.02)
    state = robot_sim.get_state()
    assert [state["x"], state["y"]] == [2, 1]
    assert state["status"] == "finished"


# ---------- 模块：算法 ----------

def test_bfs_finds_path_around_wall():
    result = path_planner.bfs(20, [[5, y] for y in range(5, 15)], [4, 9], [6, 9])
    assert result["ok"] is True
    assert result["path"][0] == [4, 9] and result["path"][-1] == [6, 9]
    assert [5, 9] not in result["path"]


def test_bfs_unreachable():
    cage = [[1, 0], [0, 1], [1, 2], [2, 1]]
    result = path_planner.bfs(20, cage, [0, 0], [1, 1])
    assert result["ok"] is False
    assert result["msg"] == "无法找到路径"


def test_patrol_skips_unreachable():
    obstacles = [[1, 1]]
    result = path_planner.patrol(20, obstacles, [[0, 0], [1, 1], [5, 5]])
    assert result[-1] == [5, 5]
    assert result.unreachable_points == [[1, 1]]


# ---------- 模块：传感器 ----------

def test_sensor_tracks_position():
    obstacles = [[6, 5]]
    assert sensor_sim.detect({"x": 5, "y": 5, "angle": 0}, obstacles)["front_blocked"] is True
    assert sensor_sim.detect({"x": 7, "y": 5, "angle": 0}, obstacles)["front_blocked"] is False
    assert sensor_sim.detect({"x": 9, "y": 9, "angle": 0}, obstacles)["out_of_bound"] is False


def test_sensor_out_of_bound_and_collision():
    assert sensor_sim.detect({"x": 20, "y": 5, "angle": 0}, [])["out_of_bound"] is True
    assert sensor_sim.detect({"x": 3, "y": 3, "angle": 0}, [[3, 3]])["collision"] is True


# ---------- 接口 ----------

def test_api_map(client):
    data = client.get("/api/map").get_json()
    assert data["size"] == 20
    assert data["start"] == [0, 0]
    assert len(data["obstacles"]) > 0


def test_api_robot(client):
    data = client.get("/api/robot").get_json()
    assert set(data) == {"x", "y", "angle", "status"}
    assert data["status"] == "idle"


def test_api_move_returns_409_on_block(client):
    robot_sim.set_map([[1, 0]], MAP_SIZE)
    resp = client.post("/api/move", json={"direction": "right"})
    assert resp.status_code == 409
    assert resp.get_json()["ok"] is False


def test_api_patrol_runs_and_reports_progress(client):
    resp = client.post("/api/patrol", json={"points": [[3, 4]], "speed": 4})
    body = resp.get_json()
    assert resp.status_code == 200
    assert body["ok"] is True and body["plan"][0] == [0, 0]

    assert wait_finished(client) == "finished"
    robot = client.get("/api/robot").get_json()
    assert [robot["x"], robot["y"]] == [3, 4]

    sensor = client.get("/api/sensor").get_json()
    assert sensor["progress"]["total"] == 1
    assert sensor["progress"]["finished"] == 1
    assert sensor["progress"]["remain"] == 0


def test_api_patrol_rejects_obstacle_point(client):
    resp = client.post("/api/patrol", json={"points": [list(sorted(BLOCKED)[0])], "speed": 1})
    assert resp.status_code == 409
    assert resp.get_json()["ok"] is False


def test_api_sensor_fields(client):
    data = client.get("/api/sensor").get_json()
    for key in ("front_blocked", "left_blocked", "right_blocked",
                "collision", "out_of_bound", "progress"):
        assert key in data


def test_api_stop(client):
    client.post("/api/patrol", json={"points": [[15, 18]], "speed": 0.5})
    time.sleep(0.2)
    client.post("/api/stop")
    time.sleep(0.1)
    assert client.get("/api/robot").get_json()["status"] == "idle"
