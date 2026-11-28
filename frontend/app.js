/* 智能巡检机器人二维仿真系统 —— 页面逻辑（Canvas 绘制、交互、轮询） */
(function () {
  "use strict";

  var SIZE = 20, PAD = 22, CELL = 40;
  var W = SIZE * CELL + PAD * 2;
  var SPEEDS = [0.5, 1, 2, 4];
  var FX = true;               // 车辆动态细节总开关：改 false 即回到纯静态小车
  var FX_FLASH_MS = 240;       // 手动单步移动后的动效短闪时长（毫秒）
  var fxUntil = 0;             // 短闪截止时间戳，performance.now() 小于它时视为“刚动过”

  var canvas = document.getElementById("map");
  var ctx = canvas.getContext("2d");
  var dpr = window.devicePixelRatio || 1;
  canvas.width = W * dpr;
  canvas.height = W * dpr;
  ctx.scale(dpr, dpr);

  var map = { size: SIZE, obstacles: [], max_patrol_points: 6 };
  var obstacles = [];
  var robot = null;
  // 位置插值：轮询 1 秒一次，直接画真实坐标会让车“每秒瞬移 2–3 格”，
  // 这里在两次采样之间做线性过渡，行进时看起来是连续移动。
  var anim = { fromX: 0, fromY: 0, toX: 0, toY: 0, fromA: 0, toA: 0, t0: 0, dur: 1000 };
  var sensor = null;
  var patrolPoints = [];
  var plan = [];
  var pathIndex = 0;
  var speed = 1;
  var speedIdx = 1;
  var lastAlert = null;
  var prevCollision = false;
  var prevOut = false;
  var prevOutOrder = [];   // 上一轮已记录的“乱序到访”点位，用于只弹一次提示
  var polling = false;

  /* ---------- 工具 ---------- */
  function api(path, options) {
    var opt = options || {};
    opt.headers = { "Content-Type": "application/json" };
    return fetch(path, opt).then(function (r) {
      return r.json().then(function (d) { return { code: r.status, data: d }; });
    });
  }

  /* ---------- 位置插值 ---------- */
  // 收到新状态时记录“上一位置 → 新位置”，draw 里按时间比例取中间点
  function setRobot(nr) {
    if (!nr) return;
    if (robot && (nr.x !== robot.x || nr.y !== robot.y || nr.angle !== robot.angle)) {
      anim.fromX = robot.x; anim.fromY = robot.y;
      anim.toX = nr.x; anim.toY = nr.y;
      anim.fromA = robot.angle || 0;
      anim.toA = nr.angle || 0;
      anim.t0 = performance.now();
    } else if (!robot) {
      anim.fromX = anim.toX = nr.x; anim.fromY = anim.toY = nr.y;
      anim.fromA = anim.toA = nr.angle || 0;
      anim.t0 = 0;
    }
    robot = nr;
  }

  function displayPos() {
    var x = robot.x, y = robot.y, angle = robot.angle || 0;
    // 只有自动巡检行进中才插值；手动移动是单格瞬移，直接画真实位置
    if (robot.status === "moving") {
      var k = Math.min(1, (performance.now() - anim.t0) / anim.dur);
      k = Math.max(0, k);
      x = anim.fromX + (anim.toX - anim.fromX) * k;
      y = anim.fromY + (anim.toY - anim.fromY) * k;
      var da = ((anim.toA - anim.fromA + 540) % 360) - 180;   // 走最短角度
      angle = anim.fromA + da * k;
    }
    return { x: x, y: y, angle: angle };
  }

  function toast(type, msg) {
    var layer = document.getElementById("alerts");
    var el = document.createElement("div");
    var title = type === "collision" ? "碰撞告警" :
                type === "out_bound" ? "越界告警" : "任务提示";
    var ico = type === "collision" ? "⛔" : type === "out_bound" ? "⚠" : "✖";
    el.className = "alert" + (type === "out_bound" || type === "plan_failed" ? " warn" : "");
    el.innerHTML =
      '<span class="ico">' + ico + "</span>" +
      '<div class="body"><span class="t">' + title + "</span>" +
      String(msg).replace(/[<>&]/g, "") + "</div>" +
      '<button class="x">✕</button>';
    el.querySelector(".x").onclick = function () { el.remove(); };
    layer.appendChild(el);
    setTimeout(function () { if (el.parentNode) el.remove(); }, 4200);

    lastAlert = { type: type, msg: msg };
    renderCards();
  }

  /* ---------- 绘制 ---------- */
  function px(i) { return PAD + i * CELL; }

  function draw() {
    ctx.clearRect(0, 0, W, W);

    var g = ctx.createRadialGradient(W / 2, W / 2, 60, W / 2, W / 2, W * 0.75);
    g.addColorStop(0, "#0b1f33");
    g.addColorStop(1, "#07121f");
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.rect(PAD, PAD, SIZE * CELL, SIZE * CELL);
    ctx.fill();

    ctx.strokeStyle = "rgba(0,229,255,0.10)";
    ctx.lineWidth = 1;
    for (var i = 0; i <= SIZE; i++) {
      ctx.beginPath();
      ctx.moveTo(px(i), PAD);
      ctx.lineTo(px(i), PAD + SIZE * CELL);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(PAD, px(i));
      ctx.lineTo(PAD + SIZE * CELL, px(i));
      ctx.stroke();
    }

    obstacles.forEach(function (o) {
      var x = px(o[0]), y = px(o[1]);
      ctx.fillStyle = "rgba(58,22,34,0.95)";
      ctx.fillRect(x + 1, y + 1, CELL - 2, CELL - 2);
      ctx.strokeStyle = "rgba(255,71,87,0.75)";
      ctx.lineWidth = 1.2;
      ctx.strokeRect(x + 1.5, y + 1.5, CELL - 3, CELL - 3);
      ctx.strokeStyle = "rgba(255,71,87,0.22)";
      ctx.beginPath();
      ctx.moveTo(x + 4, y + CELL - 4);
      ctx.lineTo(x + CELL - 4, y + 4);
      ctx.stroke();
    });

    // 已走过
    if (plan.length > 1 && pathIndex > 0) {
      ctx.save();
      ctx.shadowColor = "rgba(127,216,255,0.65)";
      ctx.shadowBlur = 8;
      ctx.strokeStyle = "rgba(127,216,255,0.95)";
      ctx.lineWidth = 3;
      ctx.lineJoin = "round";
      ctx.beginPath();
      for (var k = 0; k <= pathIndex && k < plan.length; k++) {
        var vx = px(plan[k][0]) + CELL / 2;
        var vy = px(plan[k][1]) + CELL / 2;
        if (k === 0) ctx.moveTo(vx, vy); else ctx.lineTo(vx, vy);
      }
      ctx.stroke();
      ctx.restore();
    }

    // 待行进
    if (plan.length > pathIndex + 1) {
      ctx.save();
      ctx.setLineDash([6, 6]);
      ctx.shadowColor = "rgba(0,229,255,0.6)";
      ctx.shadowBlur = 10;
      ctx.strokeStyle = "rgba(0,229,255,0.85)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (var j = pathIndex; j < plan.length; j++) {
        var nx = px(plan[j][0]) + CELL / 2;
        var ny = px(plan[j][1]) + CELL / 2;
        if (j === pathIndex) ctx.moveTo(nx, ny); else ctx.lineTo(nx, ny);
      }
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = "rgba(0,229,255,0.9)";
      for (var d = pathIndex; d < plan.length; d++) {
        ctx.beginPath();
        ctx.arc(px(plan[d][0]) + CELL / 2, px(plan[d][1]) + CELL / 2, 2.2, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 巡检点
    var target = sensor && sensor.target ? sensor.target : null;
    var visited = sensor && sensor.visited ? sensor.visited : [];
    var outOfOrder = sensor && sensor.out_of_order ? sensor.out_of_order : [];
    patrolPoints.forEach(function (p, n) {
      var cx = px(p[0]) + CELL / 2, cy = px(p[1]) + CELL / 2;
      var isTarget = target && target[0] === p[0] && target[1] === p[1];
      var isVisited = visited.some(function (v) { return v[0] === p[0] && v[1] === p[1]; });
      var isOutOrder = !isTarget && outOfOrder.some(function (v) { return v[0] === p[0] && v[1] === p[1]; });

      ctx.beginPath();
      ctx.arc(cx, cy, 10, 0, Math.PI * 2);
      ctx.fillStyle = isTarget ? "rgba(255,204,0,0.22)" :
                      isOutOrder ? "rgba(255,159,67,0.20)" :
                      isVisited ? "rgba(95,240,176,0.18)" : "rgba(0,229,255,0.14)";
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = isTarget ? "#ffcc00" :
                        isOutOrder ? "#ff9f43" :
                        isVisited ? "#5ff0b0" : "rgba(0,229,255,0.85)";
      if (isTarget) { ctx.shadowColor = "#ffcc00"; ctx.shadowBlur = 12; }
      ctx.stroke();
      ctx.shadowBlur = 0;

      ctx.fillStyle = isTarget ? "#ffcc00" :
                      isOutOrder ? "#ff9f43" :
                      isVisited ? "#5ff0b0" : "#bfefff";
      ctx.font = "bold 11px Menlo, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("P" + (n + 1), cx, cy + 0.5);
    });

    // 机器人（俯视小车，车头朝 +x，随 angle 旋转）
    if (robot) {
      var disp = displayPos();
      var cx2 = px(disp.x) + CELL / 2, cy2 = px(disp.y) + CELL / 2;
      var a = disp.angle * Math.PI / 180;
      var moving = robot.status === "moving";
      var flash = FX && performance.now() < fxUntil;   // 手动单步后的短闪
      var active = moving || flash;
      var alarm = !!(sensor && (sensor.collision || sensor.out_of_bound));
      var now = performance.now() / 1000;
      var spin = (FX && active) ? now * 8 : 0;    // 轮胎轮辐转动相位
      var blink = (Math.floor(now * (flash ? 12 : 3)) % 2) === 0; // 示警灯：常态 3Hz，短闪时 12Hz

      var bodyTop = alarm ? "#ffd0d5" : active ? "#d9fbff" : "#eef6ff";
      var bodyMid = alarm ? "#e85560" : active ? "#68d7e8" : "#a9c8e6";
      var bodyBottom = alarm ? "#9c1c26" : active ? "#1f8fa4" : "#6d94bd";
      var edge = alarm ? "#5c0a12" : active ? "#0b4351" : "#122c45";

      // 地面投影
      ctx.save();
      ctx.fillStyle = "rgba(0,0,0,0.42)";
      ctx.beginPath();
      ctx.ellipse(cx2 + 2, cy2 + 3, 19, 10, 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();

      ctx.save();
      ctx.translate(cx2, cy2);
      ctx.rotate(a);

      // 四个轮胎：深色胎面 + 细轮辋，移动时轮辐转动
      [[9.0, -7.7], [9.0, 7.7], [-12.0, -7.7], [-12.0, 7.7]].forEach(function (w) {
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(w[0] - 3.7, w[1] - 2.1, 7.4, 4.2, 1.6);
        else ctx.rect(w[0] - 3.7, w[1] - 2.1, 7.4, 4.2);
        ctx.fillStyle = "#0f2030";
        ctx.fill();
        ctx.strokeStyle = "rgba(150,185,215,0.5)";
        ctx.lineWidth = 0.9;
        ctx.stroke();

        if (FX && active) {
          ctx.save();
          ctx.translate(w[0], w[1]);
          ctx.rotate(spin);
          ctx.strokeStyle = "rgba(190,222,242,0.55)";
          ctx.lineWidth = 0.9;
          ctx.beginPath();
          ctx.moveTo(-2.6, 0); ctx.lineTo(2.6, 0);
          ctx.moveTo(0, -1.5); ctx.lineTo(0, 1.5);
          ctx.stroke();
          ctx.restore();
        }
      });

      // 车身：前窄后宽的俯视轮廓
      ctx.beginPath();
      ctx.moveTo(-14.4, -7.1);
      ctx.lineTo(6.0, -7.1);
      ctx.quadraticCurveTo(14.2, -7.1, 17.2, -4.4);
      ctx.quadraticCurveTo(18.6, -2.0, 18.6, 0);
      ctx.quadraticCurveTo(18.6, 2.0, 17.2, 4.4);
      ctx.quadraticCurveTo(14.2, 7.1, 6.0, 7.1);
      ctx.lineTo(-14.4, 7.1);
      ctx.quadraticCurveTo(-18.2, 7.1, -18.2, 3.6);
      ctx.lineTo(-18.2, -3.6);
      ctx.quadraticCurveTo(-18.2, -7.1, -14.4, -7.1);
      ctx.closePath();

      var grad = ctx.createLinearGradient(0, -7, 0, 7);
      grad.addColorStop(0, bodyBottom);
      grad.addColorStop(0.35, bodyMid);
      grad.addColorStop(0.62, bodyTop);
      grad.addColorStop(1, bodyMid);
      ctx.fillStyle = grad;
      ctx.strokeStyle = edge;
      ctx.lineWidth = 2;
      ctx.shadowColor = alarm ? "rgba(255,71,87,0.95)" :
                       moving ? "rgba(0,229,255,0.85)" : "rgba(140,200,255,0.6)";
      ctx.shadowBlur = alarm ? 16 : moving ? 15 : 7;
      ctx.fill();
      ctx.stroke();
      ctx.shadowBlur = 0;

      // 引擎盖高光（车头一侧更亮，帮助辨认前脸）
      ctx.fillStyle = "rgba(255,255,255,0.22)";
      ctx.beginPath();
      ctx.moveTo(9.0, -5.4);
      ctx.lineTo(16.0, -3.0);
      ctx.lineTo(16.0, 3.0);
      ctx.lineTo(9.0, 5.4);
      ctx.closePath();
      ctx.fill();

      // 引擎盖分缝
      ctx.strokeStyle = "rgba(10,30,48,0.35)";
      ctx.lineWidth = 1.1;
      ctx.beginPath();
      ctx.moveTo(7.0, -6.6);
      ctx.quadraticCurveTo(8.2, 0, 7.0, 6.6);
      ctx.stroke();

      // 座舱玻璃（后宽前窄的梯形）
      ctx.fillStyle = alarm ? "rgba(74,12,18,0.92)" : "rgba(16,46,66,0.94)";
      ctx.beginPath();
      ctx.moveTo(-10.6, -5.0);
      ctx.lineTo(2.6, -4.2);
      ctx.lineTo(5.6, -2.8);
      ctx.lineTo(5.6, 2.8);
      ctx.lineTo(2.6, 4.2);
      ctx.lineTo(-10.6, 5.0);
      ctx.closePath();
      ctx.fill();
      ctx.strokeStyle = alarm ? "rgba(255,150,160,0.7)" : "rgba(140,232,255,0.65)";
      ctx.lineWidth = 1.1;
      ctx.stroke();

      // 前风挡反光
      ctx.fillStyle = "rgba(190,240,255,0.35)";
      ctx.beginPath();
      ctx.moveTo(3.0, -4.0);
      ctx.lineTo(5.2, -2.7);
      ctx.lineTo(5.2, -0.4);
      ctx.lineTo(3.0, -1.4);
      ctx.closePath();
      ctx.fill();

      // 后视镜
      ctx.strokeStyle = edge;
      ctx.lineWidth = 1;
      [[5.8, -8.3], [5.8, 8.3]].forEach(function (m) {
        ctx.fillStyle = bodyMid;
        ctx.beginPath();
        ctx.ellipse(m[0], m[1], 2.8, 1.5, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      });

      // 车顶示警灯：行进中与手动单步短闪期间红蓝交替闪烁
      if (FX && (active || alarm)) {
        var beaconColor = alarm ? "#ff3b4e" : (blink ? "#ff4d5e" : "#3b8cff");
        var haloColor = alarm ? "rgba(255,59,78,0.30)" :
                        blink ? "rgba(255,77,94,0.30)" : "rgba(59,140,255,0.30)";
        var haloR = 3.8 + 1.4 * Math.sin(now * 6);
        ctx.fillStyle = haloColor;
        ctx.beginPath();
        ctx.arc(-3.8, 0, haloR, 0, Math.PI * 2);
        ctx.fill();
        ctx.save();
        ctx.shadowColor = beaconColor;
        ctx.shadowBlur = 12;
        ctx.fillStyle = beaconColor;
        ctx.beginPath();
        ctx.arc(-3.8, 0, 2.2, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }

      // 前灯
      ctx.fillStyle = "#f2fdff";
      [-4.3, 4.3].forEach(function (wy) {
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(14.6, wy - 1.6, 3.6, 3.2, 1.1);
        else ctx.rect(14.6, wy - 1.6, 3.6, 3.2);
        ctx.fill();
      });

      // 尾灯
      ctx.fillStyle = alarm ? "#ffffff" : "#ff5a67";
      [-4.7, 4.7].forEach(function (wy) {
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(-18.0, wy - 1.6, 2.4, 3.2, 0.8);
        else ctx.rect(-18.0, wy - 1.6, 2.4, 3.2);
        ctx.fill();
      });

      // 车头光束，亮度随行进轻微呼吸
      var beamA = FX ? 0.14 + 0.08 * Math.sin(now * 2.2) : 0.20;
      var beam = ctx.createLinearGradient(18.6, 0, 31, 0);
      beam.addColorStop(0, "rgba(226,250,255," + beamA.toFixed(3) + ")");
      beam.addColorStop(1, "rgba(226,250,255,0)");
      ctx.fillStyle = beam;
      ctx.beginPath();
      ctx.moveTo(18.6, -5.0);
      ctx.lineTo(31, -9);
      ctx.lineTo(31, 9);
      ctx.lineTo(18.6, 5.0);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    }

    ctx.strokeStyle = "rgba(0,229,255,0.35)";
    ctx.lineWidth = 1;
    ctx.strokeRect(PAD - 0.5, PAD - 0.5, SIZE * CELL + 1, SIZE * CELL + 1);
  }

  /* ---------- 状态卡片 ---------- */
  function setCard(id, text, cls) {
    var card = document.getElementById(id);
    card.className = "card" + (cls ? " " + cls : "");
    document.getElementById("v" + id.slice(4)).innerHTML = text;
  }

  function renderCards() {
    if (!robot) return;

    document.getElementById("vPos").innerHTML = "(" + robot.x + ", " + robot.y + ")";

    var statusText = robot.status === "moving" ? "移动中" :
                     robot.status === "finished" ? "已完成" : "空闲";
    var statusNote = robot.status === "moving" ? "自动" :
                     robot.status === "finished" ? "已到达" : "待命";
    document.getElementById("vState").innerHTML =
      statusText + "<small>" + statusNote + "</small>";

    var sensorText = "正常", sensorCls = "ok";
    if (sensor) {
      if (sensor.collision) { sensorText = "碰撞"; sensorCls = "bad"; }
      else if (sensor.out_of_bound) { sensorText = "越界"; sensorCls = "bad"; }
      else if (sensor.front_blocked) { sensorText = "前方受阻"; sensorCls = "warn"; }
      else if (sensor.left_blocked) { sensorText = "左侧受阻"; sensorCls = "warn"; }
      else if (sensor.right_blocked) { sensorText = "右侧受阻"; sensorCls = "warn"; }
    }
    setCard("cardSensor", sensorText, sensorCls);

    var alertText = "无", alertCls = "ok";
    if (lastAlert) {
      alertCls = lastAlert.type === "out_bound" || lastAlert.type === "plan_failed" ? "warn" : "bad";
      alertText = lastAlert.type === "collision" ? "碰撞告警" :
                  lastAlert.type === "out_bound" ? "越界告警" : "任务提示";
    }
    setCard("cardAlert", alertText, alertCls);

    var pg = sensor && sensor.progress ? sensor.progress
             : { finished: 0, total: 0, remain: 0, mode: "idle" };
    var note = pg.mode === "auto" ? "剩余 " + pg.remain + " 步" :
               pg.total ? "手动巡检 · 按顺序" : "未开始";
    document.getElementById("vTask").innerHTML =
      pg.finished + " / " + pg.total + "<small>" + note + "</small>";
    document.getElementById("vBar").style.width =
      (pg.total ? Math.round((pg.finished / pg.total) * 100) : 0) + "%";

    var moving = robot.status === "moving";
    document.getElementById("sysdot").className = "dot" + (moving ? " moving" : "");
    document.getElementById("systext").textContent = moving ? "机器人运行中" : "系统空闲";
  }

  function render() {
    draw();
    renderCards();
  }

  /* ---------- 进度同步 ---------- */
  function syncIndex() {
    if (!plan.length || !robot) return;
    for (var i = pathIndex; i < plan.length; i++) {
      if (plan[i][0] === robot.x && plan[i][1] === robot.y) {
        pathIndex = i;
        return;
      }
    }
  }

  /* ---------- 动画（行进中或手动短闪期间运行，否则停在最后一帧，避免空转） ---------- */
  var animOn = false;
  function fxActive() {
    return FX && performance.now() < fxUntil;
  }
  function startAnim() {
    if (animOn) return;
    animOn = true;
    var step = function () {
      draw();
      if ((robot && robot.status === "moving") || fxActive()) {
        requestAnimationFrame(step);
      } else {
        animOn = false;
      }
    };
    requestAnimationFrame(step);
  }

  /* ---------- 轮询 ---------- */
  function poll() {
    if (polling) return;
    polling = true;
    Promise.all([api("/api/robot"), api("/api/sensor")])
      .then(function (res) {
        setRobot(res[0].data);
        sensor = res[1].data;
        syncIndex();

        if (sensor.out_of_bound && !prevOut) toast("out_bound", "机器人已越出地图边界");
        if (sensor.collision && !prevCollision) toast("collision", "机器人当前位置与障碍物重合");
        prevOut = !!sensor.out_of_bound;
        prevCollision = !!sensor.collision;

        // 未按顺序到达：该点位已到访但不计入进度，提示一次
        var ooo = sensor.out_of_order || [];
        var tgt = sensor.target;
        ooo.forEach(function (p) {
          var known = prevOutOrder.some(function (q) { return q[0] === p[0] && q[1] === p[1]; });
          if (known) return;
          var pIdx = patrolPoints.findIndex(function (q) { return q[0] === p[0] && q[1] === p[1]; });
          var tIdx = tgt ? patrolPoints.findIndex(function (q) { return q[0] === tgt[0] && q[1] === tgt[1]; }) : -1;
          toast("plan_failed",
            "未按顺序到达 P" + (pIdx + 1) + "，进度不更新：" +
            (tIdx >= 0 ? "请先到达 P" + (tIdx + 1) : "请按登记顺序依次到达"));
        });
        prevOutOrder = ooo;

        render();
        if (FX && ((robot && robot.status === "moving") || fxActive())) startAnim();
      })
      .catch(function () {
        document.getElementById("systext").textContent = "后端连接中断";
        document.getElementById("sysdot").className = "dot moving";
      })
      .then(function () { polling = false; });
  }

  /* ---------- 交互 ---------- */
  function move(direction) {
    api("/api/move", { method: "POST", body: JSON.stringify({ direction: direction }) })
      .then(function (res) {
        if (!res.data.ok) {
          var msg = res.data.msg || "移动被阻止";
          var type = "out_bound";
          if (msg.indexOf("巡检进行中") >= 0) {
            type = "plan_failed";
            msg = "巡检进行中，请先点击「停止任务」再手动控制";
          } else if (msg.indexOf("障碍") >= 0) {
            type = "collision";
          }
          toast(type, msg);
        } else if (FX) {
          // 手动单步成功：触发一次短闪，让操作有即时反馈
          fxUntil = performance.now() + FX_FLASH_MS;
        }
        poll();
      })
      .catch(function () { toast("plan_failed", "无法连接后端"); });
  }

  Array.prototype.forEach.call(document.querySelectorAll(".dpad button"), function (b) {
    b.onclick = function () { move(b.dataset.dir); };
  });

  window.addEventListener("keydown", function (e) {
    var map2 = { ArrowUp: "up", ArrowDown: "down", ArrowLeft: "left", ArrowRight: "right" };
    if (map2[e.key]) { e.preventDefault(); move(map2[e.key]); }
  });

  var slider = document.getElementById("speed");
  slider.oninput = function () {
    speedIdx = parseInt(slider.value, 10);
    speed = SPEEDS[speedIdx];
    document.getElementById("speedNow").textContent = speed + "x";
  };
  // 巡检进行中拖动滑块：通知后端从当前格按新速度续走（不影响已算好的路径与进度）
  slider.onchange = function () {
    if (!robot || robot.status !== "moving") return;
    api("/api/speed", { method: "POST", body: JSON.stringify({ speed: speed }) })
      .then(function (res) {
        if (!res.data.ok) toast("plan_failed", res.data.msg || "速度调整失败");
      })
      .catch(function () {});
  };

  /* ---------- 巡检点登记（后端是进度唯一来源） ---------- */
  function pushPoints() {
    api("/api/points", { method: "POST", body: JSON.stringify({ points: patrolPoints }) })
      .then(function (res) {
        if (!res.data.ok) toast("plan_failed", res.data.msg || "巡检点同步失败");
        poll();
      })
      .catch(function () { toast("plan_failed", "无法连接后端"); });
  }

  function canvasPos(evt) {
    var rect = canvas.getBoundingClientRect();
    var scale = W / rect.width;
    var x = (evt.clientX - rect.left) * scale;
    var y = (evt.clientY - rect.top) * scale;
    return { cx: Math.floor((x - PAD) / CELL), cy: Math.floor((y - PAD) / CELL) };
  }

  canvas.addEventListener("click", function (evt) {
    var p = canvasPos(evt);
    if (p.cx < 0 || p.cy < 0 || p.cx >= SIZE || p.cy >= SIZE) return;

    var hit = -1;
    for (var i = 0; i < patrolPoints.length; i++) {
      if (patrolPoints[i][0] === p.cx && patrolPoints[i][1] === p.cy) { hit = i; break; }
    }
    if (hit >= 0) {
      patrolPoints.splice(hit, 1);
    } else {
      var blocked = obstacles.some(function (o) { return o[0] === p.cx && o[1] === p.cy; });
      if (blocked) { toast("plan_failed", "该格为障碍物，不能设置巡检点"); return; }
      var maxPts = map.max_patrol_points || 6;
      if (patrolPoints.length >= maxPts) {
        toast("plan_failed", "巡检点最多 " + maxPts + " 个");
        return;
      }
      patrolPoints.push([p.cx, p.cy]);
    }
    pushPoints();
    render();
  });

  document.getElementById("btnStart").onclick = function () {
    if (!patrolPoints.length) {
      toast("plan_failed", "请先在地图上点击添加巡检点");
      return;
    }
    api("/api/patrol", {
      method: "POST",
      body: JSON.stringify({ points: patrolPoints, speed: speed })
    }).then(function (res) {
      if (!res.data.ok) { toast("plan_failed", res.data.msg || "启动失败"); return; }
      plan = res.data.plan || [];
      pathIndex = 0;
      if (res.data.msg) toast("plan_failed", res.data.msg);
      poll();
    }).catch(function () { toast("plan_failed", "无法连接后端"); });
  };

  document.getElementById("btnStop").onclick = function () {
    api("/api/stop", { method: "POST" }).then(function () { poll(); }).catch(function () {});
  };

  document.getElementById("btnReset").onclick = function () {
    api("/api/reset", { method: "POST" }).then(function (res) {
      if (!res.data.ok) { toast("plan_failed", res.data.msg || "复位失败"); return; }
      patrolPoints = [];
      plan = [];
      pathIndex = 0;
      lastAlert = null;
      prevCollision = false;
      prevOut = false;
      prevOutOrder = [];
      fxUntil = 0;
      toast("plan_failed", "已复位：机器人回到起点，巡检点与进度已清空");
      poll();
    }).catch(function () { toast("plan_failed", "无法连接后端"); });
  };

  document.getElementById("btnClear").onclick = function () {
    patrolPoints = [];
    plan = [];
    pathIndex = 0;
    pushPoints();
    render();
  };

  /* ---------- 启动 ---------- */
  api("/api/map").then(function (res) {
    map = res.data;
    obstacles = map.obstacles || [];
    render();
  }).catch(function () {
    render();
  });

  api("/api/points").then(function (res) {
    if (res.data && res.data.ok && res.data.points) patrolPoints = res.data.points;
    render();
  }).catch(function () {});

  poll();
  // 自适应采样：行进中 250ms 一次（位置插值才不会被 1 秒采样拉出明显滞后），静止时 1s 一次
  function pollTick() {
    var gap = (robot && robot.status === "moving") ? 250 : 1000;
    anim.dur = gap;
    poll();
    setTimeout(pollTick, gap);
  }
  setTimeout(pollTick, 1000);
})();
