# -*- coding: utf-8 -*-
"""接口契约测试：九个接口、校验状态码、错误文案、复位语义。"""
import json
import time

import pytest

from app import create_app


@pytest.fixture()
def client():
    app = create_app()
    c = app.test_client()
    c.post("/api/reset")
    yield c
    c.post("/api/reset")


def post(c, path, payload=None):
    return c.post(path, data=json.dumps(payload or {}), content_type="application/json")


def test_nine_api_routes_exist(client):
    rules = {str(r.rule) for r in client.application.url_map.iter_rules()
             if str(r.rule).startswith("/api")}
    assert rules == {
        "/api/map", "/api/robot", "/api/move", "/api/points",
        "/api/patrol", "/api/speed", "/api/stop", "/api/reset", "/api/sensor",
    }


def test_map_payload(client):
    j = client.get("/api/map").get_json()
    assert j["size"] == 20
    assert j["start"] == [0, 0]
    assert j["max_patrol_points"] == 6
    assert [6, 2] in j["obstacles"]


def test_robot_and_sensor_fields(client):
    r = client.get("/api/robot").get_json()
    assert set(r) >= {"x", "y", "angle", "status"}
    s = client.get("/api/sensor").get_json()
    assert set(s) >= {"front_blocked", "left_blocked", "right_blocked",
                      "collision", "out_of_bound", "progress", "visited",
                      "out_of_order", "target", "status"}


def test_move_rejects_bad_direction(client):
    r = post(client, "/api/move", {"direction": "north"})
    assert r.status_code == 409
    assert "方向参数无效" in r.get_json()["msg"]


def test_move_blocked_by_obstacle(client):
    # (0,0) 向上越界
    r = post(client, "/api/move", {"direction": "up"})
    assert r.status_code == 409


def test_points_validation(client):
    assert post(client, "/api/points", {"points": [[20, 0]]}).status_code == 409
    assert post(client, "/api/points", {"points": [[6, 2]]}).status_code == 409          # 障碍
    assert post(client, "/api/points", {"points": "bad"}).status_code == 409
    many = [[i, 19] for i in range(7)]
    assert post(client, "/api/points", {"points": many}).status_code == 409


def test_points_empty_clears(client):
    post(client, "/api/points", {"points": [[3, 4]]})
    assert client.get("/api/points").get_json()["points"] == [[3, 4]]
    post(client, "/api/points", {"points": []})
    assert client.get("/api/points").get_json()["points"] == []


def test_patrol_rejects_bad_speed(client):
    r = post(client, "/api/patrol", {"points": [[0, 3]], "speed": 1.5})
    assert r.status_code == 409
    assert r.get_json()["msg"] == "速度只支持 0.5 / 1 / 2 / 4"
    r = post(client, "/api/patrol", {"points": [[0, 3]], "speed": "abc"})
    assert r.status_code == 409
    assert r.get_json()["msg"] == "speed 应为数字"


def test_patrol_returns_plan_and_points(client):
    r = post(client, "/api/patrol", {"points": [[0, 3], [3, 0]], "speed": 4})
    j = r.get_json()
    assert r.status_code == 200
    assert j["plan"][0] == [0, 0]
    assert j["points"] == [[0, 3], [3, 0]]
    assert j["unreachable"] == []
    client.post("/api/stop")


def test_points_change_locked_while_patrolling(client):
    post(client, "/api/points", {"points": [[0, 3]]})
    post(client, "/api/patrol", {"points": [[0, 3]], "speed": 2})
    time.sleep(0.3)
    r = post(client, "/api/points", {"points": [[0, 3], [3, 0]]})
    assert r.status_code == 409
    assert "请先停止巡检" in r.get_json()["msg"]
    client.post("/api/stop")
    assert post(client, "/api/points", {"points": [[0, 3], [3, 0]]}).status_code == 200


def test_manual_move_locked_while_patrolling(client):
    post(client, "/api/points", {"points": [[0, 3]]})
    post(client, "/api/patrol", {"points": [[0, 3]], "speed": 2})
    time.sleep(0.3)
    r = post(client, "/api/move", {"direction": "right"})
    j = r.get_json()
    assert r.status_code == 409
    assert j["msg"] == "巡检进行中，请先停止任务再手动控制"
    assert "stop()" not in j["msg"]                 # 不向用户暴露接口名
    client.post("/api/stop")


def test_speed_without_patrol_only_records(client):
    j = post(client, "/api/speed", {"speed": 2}).get_json()
    assert j["ok"] is True and j["applied"] is False


def test_speed_rejects_illegal_value(client):
    r = post(client, "/api/speed", {"speed": 3})
    assert r.status_code == 409
    assert r.get_json()["msg"] == "速度只支持 0.5 / 1 / 2 / 4"


def test_reset_zeroes_everything(client):
    post(client, "/api/points", {"points": [[1, 0], [2, 0]]})
    post(client, "/api/move", {"direction": "right"})
    j = post(client, "/api/reset").get_json()
    assert j["ok"] is True
    assert j["robot"]["x"] == 0 and j["robot"]["y"] == 0
    assert j["robot"]["status"] == "idle"
    assert j["points"] == []
    s = client.get("/api/sensor").get_json()
    assert s["progress"]["total"] == 0
    assert s["progress"]["finished"] == 0


def test_get_is_side_effect_free(client):
    """GET 不再推进进度：不登记点位时读到什么都不会变。"""
    post(client, "/api/points", {"points": [[1, 0]]})
    for _ in range(3):
        assert client.get("/api/sensor").get_json()["progress"]["finished"] == 0
        client.get("/api/robot")
    assert client.get("/api/sensor").get_json()["progress"]["finished"] == 0
