# -*- coding: utf-8 -*-
"""sim —— 智能巡检机器人二维仿真系统的三个纯逻辑模块。

    robot_sim.py    嵌入式仿真：机器人坐标、朝向、状态、移动与路径跟随
    path_planner.py 算法：BFS 寻路与多巡检点串联
    sensor_sim.py   人工智能：虚拟传感器（前/左/右探测、碰撞、越界）

三个模块只使用 Python 标准库，互不控制对方，由 backend/app.py 统一编排：

    from sim import robot_sim, path_planner, sensor_sim
"""
# sim 仿真模块包
from .robot_sim import *
from .path_planner import *
from .sensor_sim import *