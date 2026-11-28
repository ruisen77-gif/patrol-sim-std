# 运行说明

## 环境要求

- Python 3.9 及以上（容器内为 3.11）
- 可选：Docker 24 及以上

## 方式一：本地直接运行

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python backend/app.py
```

浏览器打开 http://localhost:5050

端口可通过环境变量修改：

```bash
PORT=8000 python backend/app.py
```

## 方式二：Docker（推荐，一条命令）

```bash
docker compose -f docker/docker-compose.yml up --build
```

浏览器打开 http://localhost:5050

停止：

```bash
docker compose -f docker/docker-compose.yml down
```

## 运行测试

```bash
pytest -q
```

## 冒烟测试（手工）

```bash
curl -s localhost:5050/api/map | python3 -m json.tool
curl -s -X POST localhost:5050/api/move -H 'Content-Type: application/json' -d '{"direction":"right"}'
curl -s localhost:5050/api/status | python3 -m json.tool
curl -s -X POST localhost:5050/api/start_patrol -H 'Content-Type: application/json' -d '{"patrol_points":[[3,4]],"speed_multiplier":2}'
curl -s -X POST localhost:5050/api/stop
curl -s -X POST localhost:5050/api/reset
```

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 页面显示"后端连接中断" | Flask 未启动或端口不通 | 确认 `python backend/app.py` 仍在运行，检查 PORT |
| 巡检启动后状态不更新 | `follow_path` 被同步调用 | 确认 `_patrol_worker` 在后台线程中执行 |
| 容器内 404 | 静态目录未复制 | 检查 Dockerfile 是否复制 `frontend/` |
| 容器内 ImportError | 模块目录未复制 | 检查 Dockerfile 是否复制三个 `module_*` 目录 |
| 巡检点全部不可达 | 点位被障碍物包围 | 换一个空白格重新设置 |
| 端口 5000 被占用 | macOS 的 AirPlay 接收器默认占用 5000 | 本项目已默认使用 5050，或设置 `PORT` 环境变量换个端口 |
| 构建镜像时拉取基础镜像失败 | 无法访问 Docker Hub | 先通过可用镜像源拉取并重命名：`docker pull <mirror>/library/python:3.11-slim && docker tag <mirror>/library/python:3.11-slim python:3.11-slim` |
| 容器内 pip 安装超时 | PyPI 访问慢 | 构建时指定源：`docker compose -f docker/docker-compose.yml build --build-arg PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple` |
