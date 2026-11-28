# 项目协作约定
<!--
这里可以写希望 Agent 在整个项目中遵循的约定。请按项目实际情况选择需要的内容。
- 项目目标和当前开发范围，例如本阶段计划完成基础版的哪些功能。
- 目录职责和文件位置，例如地图配置、仿真逻辑和接口说明分别放在哪里。
- 开发约定，例如文件命名、地图坐标、机器人状态字段和 API 返回格式。
- 常用命令和验证方式，例如从哪个目录启动服务，改动后怎样验证手动控制。
- 协作边界，例如哪些文件由谁维护，哪些 Git 或部署操作需要先沟通。
将确定的约定写在本注释下方。已有说明可以直接链接到 README 或 docs 中的对应章节。
-->

# AGENTS.md 小组协作开发约定
本文件为智能巡检机器人二维仿真项目的团队协作规范，全体组员开发、提交代码均需要遵守。

## 1. 小组角色与职责边界
- **全栈**：前端页面开发、Flask后端接口实现、`docs/03_项目架构与接口.md`接口契约维护、前后端接口联调。
- **算法**：`backend/sim/` 模块开发，包含机器人运动仿真、BFS路径规划、虚拟传感器逻辑。
- **运维**：Dockerfile、docker‑compose.yml容器编排、GitHub Actions CI流水线、`backend/test/`单元测试目录骨架、`AGENTS.md`以及`docs`下分工、部署文档维护。
- **人工智能**：项目需求调研、仿真功能效果测试、方案可行性验证。
- **嵌入式**：负责真实机器人硬件对接预留方案，本仿真版本无实体硬件开发。

> 重要：不同角色尽量不要修改不属于自己职责的核心代码文件，改动跨模块内容需要和对应负责人沟通。

## 2. 当前项目开发范围
本项目为20×20二维巡检仿真系统；
现阶段`backend/sim/`仿真核心模块尚未开发完成，该目录后续由算法同学交付放入；
缺少sim模块时，巡检、传感器相关业务功能无法运行，基础Web服务可以正常启动。

## 3. 目录职责约定
- `backend/sim/`：算法仿真核心业务代码
- `backend/test/`：存放单元测试用例
- `frontend/index.html`：前端HUD可视化页面
- `docs/`：全部设计、分工、接口、部署文档，**不存放业务源码**
- Dockerfile、docker‑compose.yml、AGENTS.md、README.md放置于项目根目录

## 4. Git分支管理规范
1. `main`为主稳定分支，**禁止直接在main分支提交代码**。
2. 开发新建功能分支，命名格式：`角色/功能简述`
    - 示例：`algorithm/sim-bfs`、`fullstack/web-ui`、`ops/docker-ci`
3. 开发完成提交Pull Request/MR，经过简单评审之后再合并至main。

## 5. Commit提交信息规范
格式：`类型: 简短描述`
类型可选标签：
- `feat:` 新增业务功能
- `fix:` 修复bug
- `docs:` 修改各类md文档、README
- `refactor:` 代码重构，不改变业务行为
- `test:` 单元测试新增或修改
- `ci:` GitHub Actions流水线配置变更
- `ops:` Docker、部署相关改动

## 6. 代码与命名规范
1. Python代码遵循PEP‑8编码规范，文件统一使用UTF‑8编码。
2. 重要业务函数添加注释说明用途。
3. 禁止将密码、密钥写死在代码内。
4. API接口、字段定义以 `docs/03_项目架构与接口.md` 契约为准，修改接口需要同步更新该文档。

## 7. 项目启动常用命令
本地Python启动（项目根目录执行）
```bash
# Linux / MacOS
python3 -m venv .venv
source .venv/bin/activate
pip install -r backend/requirements.txt
python backend/app.py
```

```bash
# Windows
python -m venv .venv
.venv\Scripts\activate
pip install -r backend/requirements.txt
python backend/app.py
```

Docker Compose 启动（项目根目录）

```bash
docker compose build
docker compose up
# 停止
docker compose down
```
服务默认端口：**5050**，访问地址：`http://127.0.0.1:5050`

## 8. MR / PR 合并要求

1. 提交 PR/MR 需要写明本次修改内容；
2. 尽量保证代码无语法错误、服务可启动再合并；
3. 如果改动涉及接口、部署流程，同步更新对应 md 文档。