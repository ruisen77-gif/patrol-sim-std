// 前端冒烟测试：静态资源引用完整 + app.js 语法可编译。
// 运行：node frontend/test/smoke.mjs   （在仓库根目录执行）
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import vm from "node:vm";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
let failed = 0;

function check(name, cond, detail) {
  if (cond) {
    console.log("  ok   " + name);
  } else {
    failed += 1;
    console.log("  FAIL " + name + (detail ? " -> " + detail : ""));
  }
}

const html = readFileSync(join(root, "index.html"), "utf8");
const css = readFileSync(join(root, "style.css"), "utf8");
const js = readFileSync(join(root, "app.js"), "utf8");

console.log("index.html / style.css / app.js");
check("index.html 引用了 style.css", /<link[^>]+href="style\.css"/.test(html));
check("index.html 引用了 app.js", /<script[^>]+src="app\.js"/.test(html));
check("index.html 不再内联样式", !/<style[\s>]/.test(html));
check("index.html 不再内联脚本", !/<script[\s>][^<]*\n/.test(html));
check("style.css 非空", css.trim().length > 1000, css.length + " 字节");
check("app.js 非空", js.trim().length > 5000, js.length + " 字节");

// 关键逻辑仍在：进度卡、严格顺序提示、复位、实时改速、位置插值
for (const [name, re] of [
  ["包含 /api/speed 实时改速", /api\/speed/],
  ["包含 /api/reset 复位", /api\/reset/],
  ["包含 /api/points 点位同步", /api\/points/],
  ["包含乱序提示文案", /未按顺序到达/],
  ["包含位置插值 displayPos", /function displayPos/],
  ["包含自定义轮询 pollTick", /function pollTick/],
  ["包含动效开关 FX", /var FX = /],
]) {
  check(name, re.test(js));
}

try {
  new vm.Script(js, { filename: "app.js" });
  check("app.js 语法可编译", true);
} catch (e) {
  check("app.js 语法可编译", false, e.message);
}

console.log(failed === 0 ? "\n前端冒烟测试通过" : "\n前端冒烟测试失败：" + failed + " 项");
process.exit(failed === 0 ? 0 : 1);
