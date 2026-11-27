# -*- coding: utf-8 -*-
import os
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "backend"))

from app import create_app  # noqa: E402


@pytest.fixture()
def client():
    app = create_app()
    app.config["TESTING"] = True
    c = app.test_client()
    c.post("/api/reset")
    time.sleep(0.15)
    yield c
    c.post("/api/stop")


def move(client, direction):
    resp = client.post("/api/move", json={"direction": direction})
    return resp.get_json()


def status(client):
    return client.get("/api/status").get_json()


def test_map(client):
    data = client.get("/api/map").get_json()
    assert data["size"] == 20
    assert len(data["obstacles"]) > 0
    assert data["start"] == [0, 0]


def test_manual_move(client):
    assert move(client, "right")["ok"] is True
    assert move(client, "down")["ok"] is True
    s = status(client)
    assert (s["x"], s["y"]) == (1, 1)


def test_collision_alert(client):
    move(client, "down")
    move(client, "down")
    for _ in range(5):
        move(client, "right")
    s = status(client)
    assert (s["x"], s["y"]) == (5, 2)

    result = move(client, "right")
    assert result["ok"] is False
    assert result["alert"] == "collision"

    s = status(client)
    assert s["x"] == 5
    assert s["alerts"][-1]["type"] == "collision"


def test_out_of_bound_alert(client):
    result = move(client, "up")
    assert result["ok"] is False
    assert result["alert"] == "out_bound"
    assert status(client)["alerts"][-1]["type"] == "out_bound"


def test_patrol_finishes(client):
    resp = client.post("/api/start_patrol", json={
        "patrol_points": [[3, 4]],
        "speed_multiplier": 4,
    })
    assert resp.get_json()["ok"] is True

    deadline = time.time() + 8
    while time.time() < deadline:
        s = status(client)
        if not s["patrolling"] and s["patrol_finished"] >= 1:
            break
        time.sleep(0.05)

    s = status(client)
    assert s["patrol_finished"] == 1
    assert s["patrol_total"] == 1
    assert [s["x"], s["y"]] == [3, 4]
    assert len(s["trajectory"]) > 1


def test_start_patrol_rejects_bad_point(client):
    resp = client.post("/api/start_patrol", json={
        "patrol_points": [[6, 3]],
        "speed_multiplier": 1,
    })
    assert resp.get_json()["ok"] is False


def test_stop_patrol(client):
    client.post("/api/start_patrol", json={
        "patrol_points": [[15, 16]],
        "speed_multiplier": 0.5,
    })
    time.sleep(0.2)
    client.post("/api/stop")
    time.sleep(0.1)
    s = status(client)
    assert s["patrolling"] is False
    assert s["task_status"] == "idle"


def test_status_fields(client):
    s = status(client)
    for key in ("x", "y", "angle", "is_moving", "task_status", "sensor_distance",
                "has_collision", "is_out_bound", "task_abnormal",
                "patrol_total", "patrol_finished", "remain_steps",
                "full_path", "path_index", "trajectory", "current_target"):
        assert key in s
