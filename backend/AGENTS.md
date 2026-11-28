```
# Backend Agent 协作文档
所属：智能巡检机器人二维仿真系统 patrol-sim-std
角色：后端服务代理，负责仿真业务、状态维护、接口提供

## 1. 文件归属边界
本目录 `backend/` 所有代码、配置、依赖属于后端 Agent 维护范围。
- ✅ 归属：app.py、路由逻辑、仿真状态计算、requirements.txt、后端启动脚本
- ❌ 不属于：前端页面、视图渲染、静态展示逻辑
- 改动规则：后端修改只改本目录；接口变更必须同步通知前端，双方确认口径。

## 2. 坐标与字段口径
统一采用仿真世界坐标系：
- `x`：水平方向，浮点型，单位 m
- `y`：竖直方向，浮点型，单位 m
- `theta`：机器人航向角，弧度，逆时针为正
- 机器人状态字段：
  ```json
  {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,
    "status": "idle",
    "task_id": ""
  }
```

status 可选值：`idle` 空闲、`moving` 运动中、`arrived` 到达目标、`blocked` 障碍物阻挡。

## 3. 接口格式约定

统一返回 JSON，HTTP 状态码规范：

- 200：正常
- 400：参数错误
- 404：路由/资源不存在
- 500：服务内部异常

通用成功返回模板：

```json
{
  "code": 200,
  "msg": "success",
  "data": {}
}
```

前端为静态页面访问后端接口，存在跨域场景，后端开发环境建议开启 CORS 跨域支持。

示例接口：

1. `GET /robot/status`
获取机器人当前位姿与状态

2. `POST /robot/cmd`
下发运动控制指令，body 携带目标坐标

3. `POST /nav/goal`
设置导航巡检目标点

4. `GET /map`
获取地图、障碍物数据

## 4. 常用命令

本地调试（backend 目录内）

```bash
pip install -r requirements.txt
flask run --host=0.0.0.0 --port=5050
```

项目根目录 Docker 启动整套后端：

```bash
docker compose up --build
```

查看容器日志：

```bash
docker compose logs web
```

停止服务：

```bash
docker compose down
```

## 5. 协作约束

- 后端不做页面渲染，只输出结构化数据
- 坐标、字段、枚举值修改前必须同步前端
- 端口固定：后端监听 5050
- Docker部署时服务绑定0.0.0.0，保证容器外部网络可访问
- 版本更新推送前，执行docker compose down清理旧容器，避免旧镜像残留问题