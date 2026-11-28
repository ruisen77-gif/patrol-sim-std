# -*- coding: utf-8 -*-
"""sim 模块单元测试：BFS 规划、自动巡检路径拼接、传感器读数。"""
import sys

from sim import path_planner, sensor_sim


def test_bfs_straight_line():
    res = path_planner.bfs(20, [], [0, 0], [3, 0])
    assert res["ok"] is True
    assert res["path"] == [[0, 0], [1, 0], [2, 0], [3, 0]]


def test_bfs_detour_around_wall():
    obs = [[1, 0]]
    res = path_planner.bfs(20, obs, [0, 0], [3, 0])
    assert res["ok"] is True
    assert [1, 0] not in res["path"]
    assert res["path"][0] == [0, 0] and res["path"][-1] == [3, 0]


def test_bfs_enclosed_target_unreachable():
    obs = [[1, 1], [1, 2], [2, 1]]
    assert path_planner.bfs(20, obs, [0, 0], [1, 1])["ok"] is False


def test_bfs_rejects_out_of_range():
    assert path_planner.bfs(20, [], [0, 0], [20, 0])["ok"] is False


def test_patrol_path_visits_points_in_order():
    pts = [[0, 3], [3, 0]]
    path = path_planner.patrol(20, [], pts)
    idx = [path.index(p) for p in pts]
    assert idx[0] < idx[1]
    assert path[0] == pts[0]


def test_patrol_skips_unreachable_point():
    # patrol() 以 points[0] 为路径起点；(1,1) 四邻被围死，登记在第二位 → 跳过
    obs = [[1, 1], [1, 2], [2, 1], [0, 1], [1, 0]]
    path = path_planner.patrol(20, obs, [[3, 0], [1, 1]])
    assert path.unreachable_points == [[1, 1]]
    assert [3, 0] in path


def test_sensor_reports_blocked_directions():
    state = {"x": 0, "y": 0, "angle": 90, "status": "idle"}   # 朝下
    d = sensor_sim.detect(state, [[1, 0]], 20)
    assert d["front_blocked"] is False      # 下 (0,1) 空
    assert d["right_blocked"] is True       # 朝下时右手是 (1,0) 有障碍


def test_sensor_detects_collision_and_bound():
    d = sensor_sim.detect({"x": 6, "y": 2, "angle": 0, "status": "idle"}, [[6, 2]], 20)
    assert d["collision"] is True
    d = sensor_sim.detect({"x": 20, "y": 0, "angle": 0, "status": "idle"}, [], 20)
    assert d["out_of_bound"] is True


def test_sensor_bad_input_never_raises():
    d = sensor_sim.detect({"x": "a"}, [], 20)
    assert d["collision"] is False and "msg" in d
