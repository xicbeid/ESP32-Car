/*
 * Web Control Module — HTTP server + embedded HTML + MJPEG camera stream
 *
 * Architecture:
 *   Port 80 (httpd)  — page, motor ctrl, snapshot, favicon (quick handlers)
 *   Port 81 (raw TCP) — MJPEG stream in DEDICATED task (doesn't block httpd)
 *
 * Dual control modes:
 *   D-Pad (velocity)  — press=go slow, release=stop (direct speed, no encoder)
 *   GO button (position) — encoder closed-loop to target distance at preset speed
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
#include "web_control.h"
#include "camera_module.h"

#define TAG "WebCtrl"

/* ==================== MJPEG constants ==================== */
#define MJPEG_BOUNDARY        "frame"
#define MJPEG_STREAM_BOUNDARY "\r\n--" MJPEG_BOUNDARY "\r\n"
#define MJPEG_STREAM_PART     "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"
#define MJPEG_PORT            81

/* ==================== Embedded HTML ==================== */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>ESP32-P4 小车</title>"
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

/* === D-Pad (velocity mode) === */
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

/* === Sliders === */
".ctls{width:100%;max-width:300px;margin:8px 0;padding:0 10px}"
".row{display:flex;align-items:center;justify-content:space-between;margin:6px 0}"
".row label{font-size:13px;min-width:40px}"
".row input[type=range]{flex:1;height:30px;accent-color:#e94560;margin:0 8px}"
".row .val{font-size:13px;font-weight:bold;color:#e94560;min-width:44px;text-align:right}"

/* === GO / STOP buttons === */
".acts{display:flex;gap:10px;justify-content:center;margin:6px 0;width:100%;max-width:300px}"
".go,.stop2{flex:1;padding:14px;border:none;border-radius:14px;font-size:18px;font-weight:bold;"
"color:#fff;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}"
".go{background:#27ae60}.go:active{background:#1e8449}"
".stop2{background:#c0392b}.stop2:active{background:#922b21}"
".footer{font-size:9px;color:#555;margin-top:4px}</style></head>"

"<body><h1>ESP32-P4 Camera Car</h1>"
"<div class='cam-box'><img id='camImg' alt='Camera'><span class='cam-label'>&#128247; Live</span></div>"
"<div class='st' id='st'>就绪</div>"

/* D-Pad: velocity mode — press=go slow, release=stop */
"<div class='pad'>"
"<button class='pb pu' id='bu'>&#9650;</button>"
"<button class='pb pl' id='bl'>&#9664;</button>"
"<button class='pb pc' id='bc'>STOP</button>"
"<button class='pb pr' id='br'>&#9654;</button>"
"<button class='pb pd' id='bd'>&#9660;</button>"
"</div>"

/* Sliders for GO mode */
"<div class='ctls'>"
"<div class='row'><label>距离</label>"
"<input type='range' id='di' min='5' max='500' value='50' step='5'>"
"<span class='val' id='d2'>50cm</span></div>"
"<div class='row'><label>速度</label>"
"<input type='range' id='sp' min='10' max='100' value='50' step='5'>"
"<span class='val' id='s2'>50</span></div>"
"</div>"

/* GO / STOP for position mode */
"<div class='acts'>"
"<button class='go' id='bgo'>GO</button>"
"<button class='stop2' id='bstop'>&#9632; STOP</button>"
"</div>"

"<div class='footer'>ESP32-P4 &bull; SC2336 CSI &bull; ESP-Hosted WiFi</div>"

"<script>"
"var DI=document.getElementById('di'),SP=document.getElementById('sp');"
"var bt=0,VEL_SPD=25;"  /* D-pad uses fixed slow speed */

"/* Set MJPEG source dynamically */"
"document.getElementById('camImg').src='http://'+window.location.hostname+':81/';"

"/* D-Pad: velocity mode (press=go, release=stop) */"
"function vel(c){"
"  var n=Date.now();"
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
"</script></body></html>";

/* ==================== HTTP Handlers ==================== */
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

    if (!g_motor_callback) { httpd_resp_sendstr(req, "NO_CB"); return ESP_OK; }

    /* ── Parse command ── */
    if (strcmp(cmd_str, "vel_fwd") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_FWD, 0, speed);
        httpd_resp_sendstr(req, "VEL FWD");
    } else if (strcmp(cmd_str, "vel_back") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_BACK, 0, speed);
        httpd_resp_sendstr(req, "VEL BACK");
    } else if (strcmp(cmd_str, "vel_left") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_LEFT, 0, speed);
        httpd_resp_sendstr(req, "VEL LEFT");
    } else if (strcmp(cmd_str, "vel_right") == 0) {
        g_motor_callback(MOTOR_CMD_VEL_RIGHT, 0, speed);
        httpd_resp_sendstr(req, "VEL RIGHT");
    } else if (strcmp(cmd_str, "stop") == 0) {
        g_motor_callback(MOTOR_CMD_STOP, 0, 0);
        httpd_resp_sendstr(req, "STOPPED");
    } else if (strcmp(cmd_str, "go") == 0) {
        g_motor_callback(MOTOR_CMD_GO, dist, speed);
        httpd_resp_sendstr(req, "GO");
    } else {
        httpd_resp_sendstr(req, "UNKNOWN");
    }
    return ESP_OK;
}

/* ==================== Snapshot Handler ==================== */
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

/* ==================================================================
 * Dedicated MJPEG Stream Server (port 81)
 *
 * Raw TCP socket in its own FreeRTOS task — never blocks httpd :80.
 * ================================================================== */
static void mjpeg_server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "MJPEG: socket() failed errno=%d", errno);
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
        ESP_LOGE(TAG, "MJPEG: bind(:%d) failed errno=%d", MJPEG_PORT, errno);
        close(listen_sock); vTaskDelete(NULL); return;
    }
    if (listen(listen_sock, 2) != 0) {
        ESP_LOGE(TAG, "MJPEG: listen() failed");
        close(listen_sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "MJPEG server on :%d (dedicated task)", MJPEG_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        ESP_LOGI(TAG, "MJPEG client connected");

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
        ESP_LOGI(TAG, "MJPEG client disconnected");
    }
}

/* ==================== Public API ==================== */
esp_err_t web_control_start(motor_control_cb_t callback)
{
    g_motor_callback = callback;

    /* Port 80: httpd — page, ctrl, snapshot, favicon */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;
    config.lru_purge_enable = true;
    config.max_open_sockets = 5;

    httpd_handle_t srv80 = NULL;
    config.server_port = 80;
    if (httpd_start(&srv80, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP :80 fail"); return ESP_FAIL;
    }
    httpd_uri_t u = { .method = HTTP_GET, .user_ctx = NULL };
    u.handler = favicon_handler;  u.uri = "/favicon.ico"; httpd_register_uri_handler(srv80, &u);
    u.handler = index_handler;    u.uri = "/";            httpd_register_uri_handler(srv80, &u);
    u.handler = ctrl_handler;     u.uri = "/ctrl";        httpd_register_uri_handler(srv80, &u);
    u.handler = snapshot_handler; u.uri = "/snapshot";    httpd_register_uri_handler(srv80, &u);

    /* Port 81: dedicated MJPEG stream task */
    xTaskCreate(mjpeg_server_task, "mjpeg_srv", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Control : http://192.168.4.1/");
    ESP_LOGI(TAG, "MJPEG   : http://192.168.4.1:81/");
    return ESP_OK;
}
