import sys
sys.path.append(".")

from sim import robot_sim,path_planner

def test_bfs_not_reachable():
    """测试BFS完全被障碍物包围，找不到路径"""
    obs=[[1,1],[1,2],[2,1]]
    res=path_planner.bfs(20,obs,[0,0],[1,1])
    assert res["ok"] is False

def test_robot_move_block():
    """测试机器人撞障碍物不能移动"""
    robot_sim.set_map(obstacles=[[1,0]],size=20)
    res=robot_sim.move("right")
    assert res["ok"] == False
