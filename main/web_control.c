/*
 * 网页控制模块 — HTTP 服务器 + 嵌入式 HTML + MJPEG 摄像头推流
 *
 * 架构:
 *   :80 (httpd) — 页面, 电机控制, 快照, 图标 (快速处理)
 *   :81 (原始TCP) — MJPEG 推流，专用任务 (不阻塞 httpd)
 *
 * 双模式操控:
 *   D-Pad (速度) — 按住慢走, 松手即停 (直接调速，不用编码器)
 *   GO 按钮 (位置) — 编码器闭环走到目标距离，按预设速度
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "esp_timer.h"
#include "web_control.h"
#include "camera_module.h"
#include "imu_usb.h"

#define TAG "WebCtrl"

/* ── 来自 uart.c 和 motor_module 的电机全局变量 ── */
extern int        Encoder_Now[4];
extern float      g_Speed[4];
extern uint8_t    g_recv_flag;
extern volatile bool g_velocity_active;
extern volatile bool g_position_active;
extern volatile bool g_motor_stop_flag;

/* ── MJPEG 常量 ── */
#define MJPEG_BOUNDARY        "frame"
#define MJPEG_STREAM_BOUNDARY "\r\n--" MJPEG_BOUNDARY "\r\n"
#define MJPEG_STREAM_PART     "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"
#define MJPEG_PORT            81

/* ── 嵌入式 HTML ── */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>搜救小车</title>"
"<style>*{margin:0;padding:0;box-sizing:border-box}"
"body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh;"
"display:flex;flex-direction:column;align-items:center;padding:4px}"
"h1{font-size:16px;margin:2px 0;color:#e94560}"
".cam-box{position:relative;width:100%;max-width:320px;background:#000;"
"border-radius:10px;overflow:hidden;margin:2px 0 4px;aspect-ratio:5/4;border:2px solid #0f3460}"
".cam-box img{width:100%;height:100%;object-fit:contain;display:block}"
".cam-label{position:absolute;top:4px;left:10px;font-size:10px;color:#fff;"
"background:rgba(0,0,0,.6);padding:2px 8px;border-radius:6px}"
".st{font-size:12px;color:#aaa;margin:2px 0;text-align:center;min-height:18px}"

/* === D-Pad (速度模式) === */
".pad{display:grid;grid-template-columns:70px 70px 70px;grid-template-rows:70px 70px 70px;"
"gap:4px;justify-content:center;margin:6px 0}"
".pb{border:none;border-radius:14px;font-size:28px;color:#fff; cursor:pointer;"
"display:flex;align-items:center;justify-content:center;user-select:none;"
"-webkit-tap-highlight-color:transparent;touch-action:manipulation;"
"box-shadow:0 3px 6px rgba(0,0,0,.5);transition:transform .05s}"
".pb:active{transform:scale(.9)}"
".pu{grid-column:2;grid-row:1;background:#1a5276}"
".pl{grid-column:1;grid-row:2;background:#1a5276}"
".pc{grid-column:2;grid-row:2;font-size:14px;font-weight:bold;background:#c0392b}"
".pr{grid-column:3;grid-row:2;background:#1a5276}"
".pd{grid-column:2;grid-row:3;background:#1a5276}"

/* === 滑块 === */
".ctls{width:100%;max-width:300px;margin:8px 0;padding:0 10px}"
".row{display:flex;align-items:center;justify-content:space-between;margin:6px 0}"
".row label{font-size:13px;min-width:40px}"
".row input[type=range]{flex:1;height:30px;accent-color:#e94560;margin:0 8px}"
".row .val{font-size:13px;font-weight:bold;color:#e94560;min-width:44px;text-align:right}"

/* === 前进 / 停止 按钮 === */
".acts{display:flex;gap:10px;justify-content:center;margin:6px 0;width:100%;max-width:300px}"
".go,.stop2{flex:1;padding:14px;border:none;border-radius:14px;font-size:18px;font-weight:bold;"
"color:#fff;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}"
".go{background:#27ae60}.go:active{background:#1e8449}"
".stop2{background:#c0392b}.stop2:active{background:#922b21}"
".footer{font-size:9px;color:#555;margin-top:4px}"

/* === 遥测面板 === */
"#tlm{width:100%;max-width:320px;margin:4px 0;padding:6px 8px;background:rgba(0,0,0,.7);"
"border-radius:8px;color:#fff;font-family:'Courier New',monospace;font-size:12px}"
".tl-row{display:flex;align-items:center;justify-content:space-between;margin:2px 0;gap:6px}"
".tl-badge{display:inline-block;padding:2px 8px;border-radius:10px;font-weight:bold;font-size:11px}"
".bg-fps{background:#1a73e8}.bg-vel{background:#0d904f}.bg-pos{background:#e37400}"
".bg-idle{background:#555;color:#ccc}.bg-rx{background:#c5221f;display:none}"

/* === IMU 显示 === */
".imu-row{display:flex;align-items:center;gap:6px;margin:3px 0;padding:2px 6px;"
"background:rgba(255,255,255,.04);border-radius:6px;font-size:10px}"
".imu-tag{font-size:9px;color:#888;min-width:24px;text-align:center}"
".imu-bar-box{flex:1;height:14px;background:#222;border-radius:4px;overflow:hidden;position:relative}"
".imu-bar-fill{height:100%;border-radius:4px;transition:width .2s ease;position:absolute;left:50%}"
".imu-bar-fill.roll{background:linear-gradient(to right,#e94560,#34a853)}"
".imu-bar-fill.pitch{background:linear-gradient(to right,#e94560,#4285f4)}"
".imu-val{min-width:30px;text-align:right;font-weight:bold;font-size:10px;color:#ddd}"
".imu-yaw{color:#fbbc04;font-weight:bold;font-size:11px}"

/* === IMU 扩展数据行 === */
".imu-xrow{display:flex;align-items:center;gap:4px;margin:2px 0;padding:1px 6px;"
"background:rgba(255,255,255,.03);border-radius:4px;font-size:9px;min-height:16px}"
".imu-xrow .ixl{color:#6895c0;min-width:38px;text-align:center;font-weight:bold;font-size:9px}"
".imu-xrow .ixv{color:#ccc;flex:1;text-align:right;font-family:'Courier New',monospace}"
".imu-xrow .ixu{color:#666;margin-left:3px;font-size:8px}"
".imu-xrow .ixax{font-size:7px;color:#555;margin:0 1px;min-width:10px;text-align:center}"
".imu-qrow{display:flex;gap:3px;margin:2px 0;padding:1px 4px;"
"background:rgba(255,255,255,.03);border-radius:4px}"
".imu-qrow .ql{color:#6895c0;font-size:9px;font-weight:bold;min-width:40px;text-align:center}"
".imu-qrow .qv{color:#aaa;font-size:8px;flex:1;text-align:right;font-family:'Courier New',monospace}"
"</style></head>"

"<body><h1>ESP32-P4 搜救小车</h1>"
"<div class='cam-box'><img id='camImg' alt='Camera'><span class='cam-label'>&#128247; 实时画面</span></div>"
"<div class='st' id='st'>就绪</div>"

/* === Telemetry Dashboard === */
"<div id='tlm'>"

/* Top badges */
"<div class='tl-row'>"
"<span class='tl-badge bg-fps' id='bfps'>帧率:--</span>"
"<span class='tl-badge bg-idle' id='bmode'>空闲</span>"
"<span class='tl-badge bg-rx' id='brx'>通信</span>"
"</div>"

/* IMU Attitude: Roll / Pitch bars + Yaw */
"<div class='imu-row' id='imuBox'>"
"<span class='imu-tag'>&#9969;</span>"
"<span style='font-size:9px;min-width:24px;color:#aaa'>横滚</span>"
"<div class='imu-bar-box'><div class='imu-bar-fill roll' id='barR'></div></div>"
"<span class='imu-val' id='valR'>--</span>"
"<span style='font-size:9px;min-width:24px;color:#aaa'>俯仰</span>"
"<div class='imu-bar-box'><div class='imu-bar-fill pitch' id='barP'></div></div>"
"<span class='imu-val' id='valP'>--</span>"
"<span class='imu-yaw' id='valY'>---&deg;</span>"
"</div>"

/* IMU: Accel */
"<div class='imu-xrow' id='rowAcc'><span class='ixl'>加速度</span>"
"<span class='ixax'>X</span><span class='ixv' id='vGx'>--</span><span class='ixu'>g</span>"
"<span class='ixax'>Y</span><span class='ixv' id='vGy'>--</span><span class='ixu'>g</span>"
"<span class='ixax'>Z</span><span class='ixv' id='vGz'>--</span><span class='ixu'>g</span></div>"

/* IMU: Gyro */
"<div class='imu-xrow' id='rowGyro'><span class='ixl'>角速度</span>"
"<span class='ixax'>X</span><span class='ixv' id='vWx'>--</span><span class='ixu'>°/s</span>"
"<span class='ixax'>Y</span><span class='ixv' id='vWy'>--</span><span class='ixu'>°/s</span>"
"<span class='ixax'>Z</span><span class='ixv' id='vWz'>--</span><span class='ixu'>°/s</span></div>"

/* IMU: Mag */
"<div class='imu-xrow' id='rowMag'><span class='ixl'>磁力计</span>"
"<span class='ixax'>X</span><span class='ixv' id='vMx'>--</span>"
"<span class='ixax'>Y</span><span class='ixv' id='vMy'>--</span>"
"<span class='ixax'>Z</span><span class='ixv' id='vMz'>--</span><span class='ixu'>磁</span></div>"

/* IMU: Pressure + Altitude */
"<div class='imu-xrow' id='rowPrs'><span class='ixl'>气压计</span>"
"<span class='ixv' id='vPr'>--</span><span class='ixu'>kPa</span>"
"<span class='ixv' id='vAlt'>--</span><span class='ixu'>m</span></div>"

/* IMU: Quaternion */
"<div class='imu-qrow' id='rowQuat'>"
"<span class='ql'>四元数</span>"
"<span class='qv' id='vQ0'>--</span>"
"<span class='qv' id='vQ1'>--</span>"
"<span class='qv' id='vQ2'>--</span>"
"<span class='qv' id='vQ3'>--</span></div>"

/* IMU: Temperature */
"<div class='imu-xrow' id='rowTmp'><span class='ixl'>温度</span>"
"<span class='ixv' id='vTmp'>--</span><span class='ixu'>°C</span></div>"

/* Encoder values (text-only) */
"<div class='imu-xrow' id='rowEnc'>"
"<span class='ixl'>编码器</span>"
"<span class='ixv' id='vEnc0'>--</span>"
"<span class='ixv' id='vEnc1'>--</span>"
"<span class='ixv' id='vEnc2'>--</span>"
"<span class='ixv' id='vEnc3'>--</span>"
"<span class='ixu'>脉冲</span></div>"

"</div>"

/* D-Pad: 5-button cross — all blue, ◀▶ = spot turn */
"<div class='pad'>"
"<button class='pb pu' id='bu'>&#9650;</button>"
"<button class='pb pl' id='bl'>&#9664;</button>"
"<button class='pb pc' id='bc'>停止</button>"
"<button class='pb pr' id='br'>&#9654;</button>"
"<button class='pb pd' id='bd'>&#9660;</button>"
"</div>"

/* Sliders */
"<div class='ctls'>"
"<div class='row'><label>距离</label>"
"<input type='range' id='di' min='5' max='500' value='50' step='5'>"
"<span class='val' id='d2'>50cm</span></div>"
"<div class='row'><label>速度</label>"
"<input type='range' id='sp' min='10' max='100' value='50' step='5'>"
"<span class='val' id='s2'>50</span></div>"
"</div>"

/* GO / STOP */
"<div class='acts'>"
"<button class='go' id='bgo'>前进</button>"
"<button class='stop2' id='bstop'>&#9632; 停止</button>"
"</div>"

"<div class='footer'>ESP32-P4 &bull; SC2336 CSI &bull; 10轴IMU &bull; WiFi直连</div>"

"<script>"
"var DI=document.getElementById('di'),SP=document.getElementById('sp');"
"var VEL_SPD=25;"

"/* Set MJPEG source dynamically */"
"document.getElementById('camImg').src='http://'+window.location.hostname+':81/';"

"/* ── D-Pad: velocity mode (press=go, release=stop) ── */"
"function vel(c){"
"  fetch('/ctrl?cmd='+c+'&dist=0&speed='+VEL_SPD)"
"  .then(function(r){return r.text()})"
"  .then(function(t){document.getElementById('st').textContent=t});"
"}"
"function vstop(){fetch('/ctrl?cmd=stop&dist=0&speed=0')"
"  .then(function(r){return r.text()})"
"  .then(function(t){document.getElementById('st').textContent='就绪'});}"

"var bu=document.getElementById('bu'),bl=document.getElementById('bl');"
"var br=document.getElementById('br'),bd=document.getElementById('bd');"
"var bc=document.getElementById('bc');"

"bu.addEventListener('mousedown',function(e){e.preventDefault();vel('vel_fwd');});"
"bu.addEventListener('touchstart',function(e){e.preventDefault();vel('vel_fwd');});"
"bu.addEventListener('mouseup',vstop);"
"bu.addEventListener('touchend',vstop);"
"bu.addEventListener('mouseleave',vstop);"
"bu.addEventListener('touchcancel',vstop);"

"bl.addEventListener('mousedown',function(e){e.preventDefault();vel('vel_left');});"
"bl.addEventListener('touchstart',function(e){e.preventDefault();vel('vel_left');});"
"bl.addEventListener('mouseup',vstop);"
"bl.addEventListener('touchend',vstop);"
"bl.addEventListener('mouseleave',vstop);"
"bl.addEventListener('touchcancel',vstop);"

"br.addEventListener('mousedown',function(e){e.preventDefault();vel('vel_right');});"
"br.addEventListener('touchstart',function(e){e.preventDefault();vel('vel_right');});"
"br.addEventListener('mouseup',vstop);"
"br.addEventListener('touchend',vstop);"
"br.addEventListener('mouseleave',vstop);"
"br.addEventListener('touchcancel',vstop);"

"bd.addEventListener('mousedown',function(e){e.preventDefault();vel('vel_back');});"
"bd.addEventListener('touchstart',function(e){e.preventDefault();vel('vel_back');});"
"bd.addEventListener('mouseup',vstop);"
"bd.addEventListener('touchend',vstop);"
"bd.addEventListener('mouseleave',vstop);"
"bd.addEventListener('touchcancel',vstop);"

"bc.addEventListener('click',function(e){e.preventDefault();vstop();});"

"/* GO button: encoder closed-loop */"
"document.getElementById('bgo').addEventListener('click',function(e){"
"  e.preventDefault();"
"  var d=DI.value,s=SP.value;"
"  document.getElementById('st').textContent='前进 '+d+'cm @'+s;"
"  fetch('/ctrl?cmd=go&dist='+d+'&speed='+s)"
"  .then(function(r){return r.text()})"
"  .then(function(t){document.getElementById('st').textContent=t});"
"});"

"/* STOP button */"
"document.getElementById('bstop').addEventListener('click',function(e){"
"  e.preventDefault();"
"  fetch('/ctrl?cmd=stop&dist=0&speed=0')"
"  .then(function(r){return r.text()})"
"  .then(function(t){document.getElementById('st').textContent='已停止'});"
"});"

"/* Slider labels */"
"DI.oninput=function(){document.getElementById('d2').textContent=DI.value+'cm'};"
"SP.oninput=function(){document.getElementById('s2').textContent=SP.value};"

/* === Dashboard polling === */
"(function(){"
"var elFps=document.getElementById('bfps'),elMode=document.getElementById('bmode');"
"var elRx=document.getElementById('brx');"
"var elImuBox=document.getElementById('imuBox');"
"var elBarR=document.getElementById('barR'),elBarP=document.getElementById('barP');"
"var elValR=document.getElementById('valR'),elValP=document.getElementById('valP'),elValY=document.getElementById('valY');"
"var ANGLE_RANGE=45;"

/* IMU extra display elements */
"var evGx=document.getElementById('vGx'), evGy=document.getElementById('vGy'), evGz=document.getElementById('vGz');"
"var evWx=document.getElementById('vWx'), evWy=document.getElementById('vWy'), evWz=document.getElementById('vWz');"
"var evMx=document.getElementById('vMx'), evMy=document.getElementById('vMy'), evMz=document.getElementById('vMz');"
"var evPr=document.getElementById('vPr'), evAlt=document.getElementById('vAlt');"
"var evQ0=document.getElementById('vQ0'), evQ1=document.getElementById('vQ1');"
"var evQ2=document.getElementById('vQ2'), evQ3=document.getElementById('vQ3');"
"var evTmp=document.getElementById('vTmp');"
"var evEnc0=document.getElementById('vEnc0'),evEnc1=document.getElementById('vEnc1');"
"var evEnc2=document.getElementById('vEnc2'),evEnc3=document.getElementById('vEnc3');"
"var imuRows=document.querySelectorAll('.imu-xrow,.imu-qrow');"

"var MODE_MAP={"
"velocity:{l:'速度',c:'bg-vel'},"
"idle:{l:'空闲',c:'bg-idle'},"
"position:{l:'位置',c:'bg-pos'}};"

"function angBar(v){"
"var x=50+(v/ANGLE_RANGE)*50;"
"return (x<0?0:x>100?100:x).toFixed(1)+'%';"
"}"

"function upd(d){"
"elFps.textContent='帧率:'+d.fps;"
"var mi=MODE_MAP[d.mode]||{l:d.mode,c:'bg-idle'};"
"elMode.textContent=mi.l;elMode.className='tl-badge '+mi.c;"
"elRx.style.display=d.recv?'inline-block':'none';"

"/* Encoder */"
"var m=d.m;"
"evEnc0.textContent=m[0].e; evEnc1.textContent=m[1].e;"
"evEnc2.textContent=m[2].e; evEnc3.textContent=m[3].e;"

"/* IMU update */"
"var z=d.imu,ok=z.ok;"
"var op=ok?'1':'.3';"
"elImuBox.style.opacity=op;"
"for(var i=0;i<imuRows.length;i++)imuRows[i].style.opacity=op;"

"/* Attitude bars */"
"elBarR.style.width=ok?angBar(z.r):'50%';"
"elBarP.style.width=ok?angBar(z.p):'50%';"
"elValR.textContent=ok?z.r.toFixed(1)+'°':'--';"
"elValP.textContent=ok?z.p.toFixed(1)+'°':'--';"
"elValY.textContent=ok?z.y.toFixed(0)+'°':'---°';"
"elValR.style.color=Math.abs(z.r)>30?'#e94560':'#ddd';"
"elValP.style.color=Math.abs(z.p)>30?'#e94560':'#ddd';"

"/* Accel */"
"evGx.textContent=ok?z.ax.toFixed(3):'--';"
"evGy.textContent=ok?z.ay.toFixed(3):'--';"
"evGz.textContent=ok?z.az.toFixed(3):'--';"

"/* Gyro */"
"evWx.textContent=ok?z.gx.toFixed(2):'--';"
"evWy.textContent=ok?z.gy.toFixed(2):'--';"
"evWz.textContent=ok?z.gz.toFixed(2):'--';"

"/* Magnetometer */"
"evMx.textContent=ok?z.mx:'--';"
"evMy.textContent=ok?z.my:'--';"
"evMz.textContent=ok?z.mz:'--';"

"/* Pressure (Pa→kPa) + Altitude (cm→m) */"
"evPr.textContent=ok?(z.pr/1000).toFixed(1):'--';"
"evAlt.textContent=ok?(z.alt/100).toFixed(1):'--';"

"/* Quaternion */"
"evQ0.textContent=ok?z.q0.toFixed(3):'--';"
"evQ1.textContent=ok?z.q1.toFixed(3):'--';"
"evQ2.textContent=ok?z.q2.toFixed(3):'--';"
"evQ3.textContent=ok?z.q3.toFixed(3):'--';"

"/* Temperature */"
"evTmp.textContent=ok?z.tmp.toFixed(1):'--';"
"}"

"function poll(){"
"var x=new XMLHttpRequest();"
"x.open('GET','/status',true);x.timeout=400;"
"x.onload=function(){if(x.status===200){try{upd(JSON.parse(x.responseText));}catch(e){}}};"
"x.onerror=function(){};x.ontimeout=function(){};"
"x.send();"
"}"
"setInterval(poll,200);poll();"
"})();"
"</script></body></html>";

/* ── HTTP 处理函数 ── */
static motor_control_cb_t g_motor_callback = NULL;

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ctrl_handler(httpd_req_t *req)
{
    char buf[128];
    char cmd_str[16] = {0}, dist_str[16] = {0}, speed_str[16] = {0};
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > sizeof(buf)) { httpd_resp_send_500(req); return ESP_FAIL; }
    if (httpd_req_get_url_query_str(req, buf, buf_len) != ESP_OK) { httpd_resp_send_500(req); return ESP_FAIL; }
    httpd_query_key_value(buf, "cmd", cmd_str, sizeof(cmd_str));
    httpd_query_key_value(buf, "dist", dist_str, sizeof(dist_str));
    httpd_query_key_value(buf, "speed", speed_str, sizeof(speed_str));
    int dist  = dist_str[0]  ? atoi(dist_str)  : 50;
    int speed = speed_str[0] ? atoi(speed_str) : 50;
    if (dist  <= 0) dist  = 50;
    if (speed <= 0) speed = 50;

    if (!g_motor_callback) { httpd_resp_sendstr(req, "未就绪"); return ESP_OK; }

    if (strcmp(cmd_str, "vel_fwd") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_FWD, 0, speed);
        httpd_resp_sendstr(req, "前进");
    } else if (strcmp(cmd_str, "vel_back") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_BACK, 0, speed);
        httpd_resp_sendstr(req, "后退");
    } else if (strcmp(cmd_str, "vel_left") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_LEFT, 0, speed);
        httpd_resp_sendstr(req, "左转");
    } else if (strcmp(cmd_str, "vel_right") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_RIGHT, 0, speed);
        httpd_resp_sendstr(req, "右转");
    } else if (strcmp(cmd_str, "stop") == 0) {
        g_motor_callback(MOTOR_CMD_STOP, 0, 0);
        httpd_resp_sendstr(req, "已停止");
    } else if (strcmp(cmd_str, "go") == 0) {
        g_motor_callback(MOTOR_CMD_GO, dist, speed);
        httpd_resp_sendstr(req, "执行");
    } else {
        httpd_resp_sendstr(req, "未知指令");
    }
    return ESP_OK;
}

/* ── 快照处理 ── */
static esp_err_t snapshot_handler(httpd_req_t *req)
{
    const uint8_t *jpeg_buf = NULL;
    size_t jpeg_len = 0;
    if (camera_module_get_frame(&jpeg_buf, &jpeg_len) != ESP_OK || jpeg_len == 0) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)jpeg_buf, (int)jpeg_len);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════ *
 *  专用 MJPEG 推流服务器 (:81) — 独立 FreeRTOS 任务
 *  原始 TCP socket，不阻塞 httpd :80
 * ═══════════════════════════════════════════════════════════════ */
static void mjpeg_server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "MJPEG: socket() 失败 errno=%d", errno);
        vTaskDelete(NULL); return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = {
        .sin_family = AF_INET, .sin_port = htons(MJPEG_PORT), .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "MJPEG: bind(:%d) 失败 errno=%d", MJPEG_PORT, errno);
        close(listen_sock); vTaskDelete(NULL); return;
    }
    if (listen(listen_sock, 2) != 0) {
        ESP_LOGE(TAG, "MJPEG: listen() 失败");
        close(listen_sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "MJPEG 服务器启动 :%d (专用任务)", MJPEG_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        ESP_LOGI(TAG, "MJPEG 客户端已连接");

        char hdr[256];
        int hdr_len = snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY "\r\n"
            "Cache-Control: no-store, must-revalidate\r\n"
            "Connection: close\r\n"
            "Pragma: no-cache\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "X-Content-Type-Options: nosniff\r\n\r\n");
        send(client_sock, hdr, hdr_len, 0);

        char part_buf[128];
        while (1) {
            const uint8_t *jpeg_buf = NULL;
            size_t jpeg_len = 0;
            if (camera_module_get_frame(&jpeg_buf, &jpeg_len) != ESP_OK || jpeg_len == 0)
                continue;
            int ph_len = snprintf(part_buf, sizeof(part_buf), MJPEG_STREAM_PART, (unsigned int)jpeg_len);
            if (send(client_sock, MJPEG_STREAM_BOUNDARY, strlen(MJPEG_STREAM_BOUNDARY), 0) < 0) break;
            if (send(client_sock, part_buf, ph_len, 0) < 0) break;
            if (send(client_sock, (const char *)jpeg_buf, (int)jpeg_len, 0) < 0) break;
        }
        close(client_sock);
        ESP_LOGI(TAG, "MJPEG 客户端已断开");
    }
}

/* ── 状态 JSON 处理 ── */
static const char *motor_mode_str(void)
{
    if (g_velocity_active)     return "velocity";
    if (g_position_active)     return "position";
    return "idle";
}

static esp_err_t status_handler(httpd_req_t *req)
{
    float    brightness = 0.0f;
    int32_t  exp_100us  = 0;
    int32_t  gain_idx   = 0;
    uint32_t fps        = camera_module_get_fps();
    int64_t  uptime_ms  = esp_timer_get_time() / 1000;
    imu_data_t imu;

    camera_module_get_ae_status(&brightness, &exp_100us, &gain_idx);
    imu_usb_get_data(&imu);

    char buf[1280];
    int len = snprintf(buf, sizeof(buf),
        "{"
        "\"t\":%lld,"
        "\"fps\":%lu,"
        "\"ae\":{\"b\":%.1f,\"e\":%ld,\"g\":%ld},"
        "\"m\":["
          "{\"e\":%d,\"s\":%.1f},"
          "{\"e\":%d,\"s\":%.1f},"
          "{\"e\":%d,\"s\":%.1f},"
          "{\"e\":%d,\"s\":%.1f}"
        "],"
        "\"mode\":\"%s\","
        "\"recv\":%u,"
        "\"imu\":{"
          "\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,"
          "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
          "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
          "\"mx\":%d,\"my\":%d,\"mz\":%d,"
          "\"pr\":%.0f,\"alt\":%.1f,"
          "\"q0\":%.4f,\"q1\":%.4f,\"q2\":%.4f,\"q3\":%.4f,"
          "\"tmp\":%.1f,"
          "\"ok\":%d"
        "}"
        "}",
        uptime_ms, (unsigned long)fps,
        (double)brightness, (long)exp_100us, (long)gain_idx,
        Encoder_Now[0], (double)g_Speed[0],
        Encoder_Now[1], (double)g_Speed[1],
        Encoder_Now[2], (double)g_Speed[2],
        Encoder_Now[3], (double)g_Speed[3],
        motor_mode_str(),
        (unsigned)g_recv_flag,
        (double)imu.roll, (double)imu.pitch, (double)imu.yaw,
        (double)imu.acc_x, (double)imu.acc_y, (double)imu.acc_z,
        (double)imu.gyro_x, (double)imu.gyro_y, (double)imu.gyro_z,
        (int)imu.mag_x, (int)imu.mag_y, (int)imu.mag_z,
        (double)imu.pressure, (double)imu.altitude,
        (double)imu.quat_q0, (double)imu.quat_q1, (double)imu.quat_q2, (double)imu.quat_q3,
        (double)imu.temperature,
        imu_usb_is_connected() ? 1 : 0
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, buf, (len > 0 && len < (int)sizeof(buf)) ? len : 0);
    return ESP_OK;
}

/* ── 公开 API ── */
esp_err_t web_control_start(motor_control_cb_t callback)
{
    g_motor_callback = callback;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;
    config.lru_purge_enable = true;
    config.max_open_sockets = 5;

    httpd_handle_t srv80 = NULL;
    config.server_port = 80;
    if (httpd_start(&srv80, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP :80 启动失败"); return ESP_FAIL;
    }
    httpd_uri_t u = { .method = HTTP_GET, .user_ctx = NULL };
    u.handler = favicon_handler;  u.uri = "/favicon.ico"; httpd_register_uri_handler(srv80, &u);
    u.handler = index_handler;    u.uri = "/";            httpd_register_uri_handler(srv80, &u);
    u.handler = ctrl_handler;     u.uri = "/ctrl";        httpd_register_uri_handler(srv80, &u);
    u.handler = snapshot_handler; u.uri = "/snapshot";    httpd_register_uri_handler(srv80, &u);
    u.handler = status_handler;   u.uri = "/status";      httpd_register_uri_handler(srv80, &u);

    xTaskCreate(mjpeg_server_task, "mjpeg_srv", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "控制页 : http://192.168.4.1/");
    ESP_LOGI(TAG, "推流   : http://192.168.4.1:81/");
    return ESP_OK;
}
