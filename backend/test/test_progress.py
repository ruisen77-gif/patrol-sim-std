# -*- coding: utf-8 -*-
"""进度口径（PatrolTracker）单元测试：严格按登记顺序、乱序不追认、增删点位重算。"""
import json
import time

import pytest

from app import create_app
from sim import robot_sim


@pytest.fixture()
def client():
    """每个用例都从干净状态开始：复位机器人 + 清空点位。"""
    app = create_app()
    c = app.test_client()
    c.post("/api/reset")
    yield c
    c.post("/api/reset")


def move(c, *dirs):
    for d in dirs:
        c.post("/api/move", data=json.dumps({"direction": d}), content_type="application/json")


def progress(c):
    j = c.get("/api/sensor").get_json()
    return j["progress"]["finished"], j["progress"]["total"], j


def set_points(c, points):
    r = c.post("/api/points", data=json.dumps({"points": points}), content_type="application/json")
    assert r.status_code == 200
    return r.get_json()


def test_counts_only_in_order(client):
    set_points(client, [[1, 0], [2, 0], [3, 0]])
    assert progress(client)[:2] == (0, 3)
    move(client, "right")
    assert progress(client)[:2] == (1, 3)
    move(client, "right", "right")
    assert progress(client)[:2] == (3, 3)


def test_out_of_order_not_counted_nor_owned(client):
    set_points(client, [[0, 3], [0, 5], [3, 0]])
    # 先跑到 P3（乱序）
    move(client, *(["right"] * 3))
    finished, total, j = progress(client)
    assert (finished, total) == (0, 3)
    assert j["out_of_order"] == [[3, 0]]
    # 补齐 P1、P2：追认？不追认，最多到 2
    move(client, *(["left"] * 3))
    move(client, *(["down"] * 5))
    finished, total, j = progress(client)
    assert (finished, total) == (2, 3)
    assert j["out_of_order"] == [[3, 0]]      # P3 仍算乱序
    assert j["target"] == [3, 0]              # 目标回到 P3
    # 重新按顺序走到 P3 才计入
    move(client, "right", "right", "right", "up", "up", "up", "up", "up")
    assert progress(client)[:2] == (3, 3)


def test_remove_counted_point_rolls_back(client):
    set_points(client, [[1, 0], [2, 0], [3, 0]])
    move(client, "right", "right")
    assert progress(client)[:2] == (2, 3)
    set_points(client, [[2, 0], [3, 0]])       # 删掉已计入的 P1
    finished, total, j = progress(client)
    assert (finished, total) == (0, 2)          # 不做“虚高保留”
    assert j["visited"] == [[2, 0]]


def test_append_point_keeps_progress(client):
    set_points(client, [[1, 0], [2, 0]])
    move(client, "right", "right")
    assert progress(client)[:2] == (2, 2)
    set_points(client, [[1, 0], [2, 0], [3, 0]])
    assert progress(client)[:2] == (2, 3)


def test_manual_mode_reports_manual(client):
    set_points(client, [[1, 0]])
    _, _, j = progress(client)
    assert j["progress"]["mode"] == "manual"
    assert j["target"] == [1, 0]


def test_auto_patrol_reaches_total(client):
    r = client.post("/api/patrol", data=json.dumps({"points": [[0, 6], [5, 0]], "speed": 4}),
                    content_type="application/json")
    assert r.status_code == 200
    deadline = time.time() + 15
    while time.time() < deadline:
        finished, total, j = progress(client)
        if finished == total and j["status"] == "finished":
            break
        time.sleep(0.2)
    finished, total, j = progress(client)
    assert (finished, total) == (2, 2)
    assert j["status"] == "finished"


def test_progress_advances_without_polling(client):
    """GET 已改为纯读，进度由后台采样线程推进，不轮询也能走完。"""
    client.post("/api/patrol", data=json.dumps({"points": [[0, 3]], "speed": 4}),
                content_type="application/json")
    time.sleep(2.0)                            # 期间不调任何接口
    tracker = client.application.config["ENGINE"]
    snap = tracker.snapshot()
    assert (snap["finished"], snap["total"]) == (1, 1)


def test_stop_then_manual_does_not_skip(client):
    """stop 后残留旧 plan 不应让手动移动跳着算进度。"""
    client.post("/api/patrol", data=json.dumps({"points": [[0, 5]], "speed": 4}),
                content_type="application/json")
    time.sleep(0.3)
    client.post("/api/stop")
    finished, total, _ = progress(client)
    before = finished
    move(client, "up", "up", "up")             # 远离路径，乱走
    after = progress(client)[0]
    assert after == before
