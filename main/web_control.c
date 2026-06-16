/*
 * Web Control Module — HTTP server + embedded HTML + MJPEG camera stream
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "web_control.h"
#include "camera_module.h"

#define TAG "WebCtrl"

/* ==================== Embedded HTML (motor control + camera live view) ==================== */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>ESP32-P4 小车</title>"
"<style>*{margin:0;padding:0;box-sizing:border-box}"
"body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh;"
"display:flex;flex-direction:column;align-items:center;padding:6px}"
"h1{font-size:17px;margin:2px 0;color:#e94560}"
".cam-box{position:relative;width:100%;max-width:340px;background:#000;"
"border-radius:10px;overflow:hidden;margin:4px 0 6px;aspect-ratio:5/4;border:2px solid #0f3460}"
".cam-box img{width:100%;height:100%;object-fit:contain;display:block}"
".cam-label{position:absolute;top:4px;left:10px;font-size:10px;color:#fff;"
"background:rgba(0,0,0,.6);padding:2px 8px;border-radius:6px}"
".s{font-size:12px;color:#ccc;margin:2px 0;text-align:center}"
".dp{display:grid;grid-template-columns:80px 80px 80px;grid-template-rows:80px 80px 80px;gap:4px;justify-content:center;margin:8px 0}"
".b{border:none;border-radius:12px;font-size:26px;color:#fff;background:#16213e;display:flex;align-items:center;justify-content:center;cursor:pointer;"
"box-shadow:0 4px 8px rgba(0,0,0,.4);user-select:none;-webkit-tap-highlight-color:transparent}"
".b:active{transform:scale(.93)}.u{grid-column:2;grid-row:1;background:#0f3460}"
".l{grid-column:1;grid-row:2;background:#0f3460}.m{grid-column:2;grid-row:2;font-size:15px;background:#e94560;font-weight:bold}"
".r{grid-column:3;grid-row:2;background:#0f3460}.d{grid-column:2;grid-row:3;background:#0f3460}"
"label{font-size:12px}input[type=range]{width:260px;height:28px;accent-color:#e94560}"
".v{color:#e94560;font-weight:bold}.x{width:260px;padding:10px;border:none;border-radius:12px;font-size:17px;font-weight:bold;color:#fff;background:#c0392b;margin-top:4px;cursor:pointer}"
".footer{font-size:9px;color:#555;margin-top:6px}</style></head>"
"<body><h1>ESP32-P4 Camera Car</h1>"
"<div class='cam-box'><img src='http://192.168.4.1:8000/stream' alt='Camera' id='camImg'><span class='cam-label'>&#128247; Live</span></div>"
"<div class='s' id='st'>就绪</div>"
"<div class='dp'><button class='b u' onmousedown='go(\"fwd\")' ontouchstart='go(\"fwd\")'>&#9650;</button>"
"<button class='b l' onmousedown='go(\"left\")' ontouchstart='go(\"left\")'>&#9664;</button>"
"<button class='b m' onclick='go(\"stop\")'>STOP</button>"
"<button class='b r' onmousedown='go(\"right\")' ontouchstart='go(\"right\")'>&#9654;</button>"
"<button class='b d' onmousedown='go(\"back\")' ontouchstart='go(\"back\")'>&#9660;</button></div>"
"<label>距离 <span class='v' id='d2'>50cm</span></label><br>"
"<input type='range' id='di' min='5' max='500' value='50' step='5'><br>"
"<label>速度 <span class='v' id='s2'>50</span></label><br>"
"<input type='range' id='sp' min='10' max='100' value='50' step='5'><br>"
"<button class='x' onclick='go(\"stop\")'>&#9632; STOP &#9632;</button>"
"<div class='footer'>ESP32-P4 &bull; SC2336 CSI &bull; ESP-Hosted WiFi</div>"
"<script>var DI=document.getElementById('di'),SP=document.getElementById('sp'),bt=0;"
"function go(c){var n=Date.now();if(c!='stop'&&n-bt<200)return;bt=n;if(c=='stop')bt=0;"
"fetch('/ctrl?cmd='+c+'&dist='+DI.value+'&speed='+SP.value).then(function(r){return r.text()}).then(function(t){document.getElementById('st').textContent=t})}"
"DI.oninput=function(){document.getElementById('d2').textContent=DI.value+'cm'};"
"SP.oninput=function(){document.getElementById('s2').textContent=SP.value};"
"</script></body></html>";

/* ==================== HTTP Handlers ==================== */
static motor_control_cb_t g_motor_callback = NULL;

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
    int dist = dist_str[0] ? atoi(dist_str) : 50;
    int speed = speed_str[0] ? atoi(speed_str) : 50;
    if (dist <= 0) dist = 50;
    if (speed <= 0) speed = 50;
    ESP_LOGI(TAG, "CTRL: cmd=%s dist=%d speed=%d", cmd_str, dist, speed);

    if (g_motor_callback) {
        if (strcmp(cmd_str, "fwd") == 0)      { g_motor_callback(MOTOR_CMD_FORWARD, dist, speed); httpd_resp_sendstr(req, "FWD OK"); }
        else if (strcmp(cmd_str, "back") == 0) { g_motor_callback(MOTOR_CMD_BACKWARD, dist, speed); httpd_resp_sendstr(req, "BACK OK"); }
        else if (strcmp(cmd_str, "left") == 0) { g_motor_callback(MOTOR_CMD_LEFT, dist, speed); httpd_resp_sendstr(req, "LEFT OK"); }
        else if (strcmp(cmd_str, "right") == 0){ g_motor_callback(MOTOR_CMD_RIGHT, dist, speed); httpd_resp_sendstr(req, "RIGHT OK"); }
        else if (strcmp(cmd_str, "stop") == 0) { g_motor_callback(MOTOR_CMD_STOP, 0, 0); httpd_resp_sendstr(req, "STOPPED"); }
        else httpd_resp_sendstr(req, "UNKNOWN");
    } else httpd_resp_sendstr(req, "NO_CB");
    return ESP_OK;
}

/* ==================== MJPEG Stream Handler ==================== */
#define MJPEG_BOUNDARY "frame"
#define MJPEG_STREAM_BOUNDARY  "\r\n--" MJPEG_BOUNDARY "\r\n"
#define MJPEG_STREAM_PART      "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

static esp_err_t stream_handler(httpd_req_t *req)
{
    char part_buf[128];
    const uint8_t *jpeg_buf = NULL;
    size_t jpeg_len = 0;

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (1) {
        if (camera_module_get_frame(&jpeg_buf, &jpeg_len) != ESP_OK || jpeg_len == 0)
            continue;

        int hdr_len = snprintf(part_buf, sizeof(part_buf), MJPEG_STREAM_PART, (unsigned int)jpeg_len);
        if (httpd_resp_send_chunk(req, MJPEG_STREAM_BOUNDARY, strlen(MJPEG_STREAM_BOUNDARY)) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, part_buf, hdr_len) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, (const char *)jpeg_buf, (int)jpeg_len) != ESP_OK) break;
    }

    ESP_LOGI(TAG, "Stream client disconnected");
    return ESP_OK;
}

/* ==================== Snapshot Handler ==================== */
static esp_err_t snapshot_handler(httpd_req_t *req)
{
    const uint8_t *jpeg_buf = NULL;
    size_t jpeg_len = 0;

    if (camera_module_get_frame(&jpeg_buf, &jpeg_len) != ESP_OK || jpeg_len == 0) {
        /* No fresh frame within 100ms — send last-known if any */
        httpd_resp_send_500(req); return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)jpeg_buf, (int)jpeg_len);
    return ESP_OK;
}

/* ==================== Public API ==================== */
esp_err_t web_control_start(motor_control_cb_t callback)
{
    g_motor_callback = callback;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;
    config.lru_purge_enable = true;

    /* --- Main server :80 — page, motor control, snapshot --- */
    httpd_handle_t srv80 = NULL;
    config.server_port = 80;
    if (httpd_start(&srv80, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP :80 fail"); return ESP_FAIL;
    }
    httpd_uri_t u = { .method = HTTP_GET, .user_ctx = NULL };
    u.handler = index_handler;    u.uri = "/";        httpd_register_uri_handler(srv80, &u);
    u.handler = ctrl_handler;     u.uri = "/ctrl";    httpd_register_uri_handler(srv80, &u);
    u.handler = snapshot_handler; u.uri = "/snapshot";httpd_register_uri_handler(srv80, &u);

    /* --- Stream server :8000 — MJPEG only, separate task —--- */
    httpd_handle_t srvStream = NULL;
    config.server_port = 8000;
    config.lru_purge_enable = false;
    if (httpd_start(&srvStream, &config) == ESP_OK) {
        httpd_uri_t su = { .uri = "/stream", .method = HTTP_GET,
                           .handler = stream_handler, .user_ctx = NULL };
        httpd_register_uri_handler(srvStream, &su);
        ESP_LOGI(TAG, "Stream: http://192.168.4.1:8000/stream");
    } else {
        ESP_LOGW(TAG, "HTTP :8000 fail — stream on :80");
        u.handler = stream_handler; u.uri = "/stream";
        httpd_register_uri_handler(srv80, &u);
    }

    ESP_LOGI(TAG, "Control: http://192.168.4.1/");
    return ESP_OK;
}
