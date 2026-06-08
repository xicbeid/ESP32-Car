/*
 * Web Control Module — HTTP server + embedded HTML (motor only)
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "web_control.h"

#define TAG "WebCtrl"

/* ==================== Embedded HTML ==================== */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Car</title>"
"<style>*{margin:0;padding:0;box-sizing:border-box}body{background:#1a1a2e;color:#eee;padding:10px;text-align:center}"
"h1{font-size:18px;color:#e94560;margin:6px 0}.s{font-size:13px;color:#ccc;margin:6px 0}"
".dp{display:grid;grid-template-columns:80px 80px 80px;grid-template-rows:80px 80px 80px;gap:4px;justify-content:center;margin:10px 0}"
".b{border:none;border-radius:12px;font-size:26px;color:#fff;background:#16213e;display:flex;align-items:center;justify-content:center;cursor:pointer}"
".b:active{transform:scale(.9)}.u{grid-column:2;grid-row:1;background:#0f3460}"
".l{grid-column:1;grid-row:2;background:#0f3460}.m{grid-column:2;grid-row:2;font-size:16px;background:#e94560;font-weight:bold}"
".r{grid-column:3;grid-row:2;background:#0f3460}.d{grid-column:2;grid-row:3;background:#0f3460}"
"label{font-size:13px}input[type=range]{width:260px;height:30px;accent-color:#e94560}"
".v{color:#e94560;font-weight:bold}.x{width:260px;padding:12px;border:none;border-radius:12px;font-size:18px;font-weight:bold;color:#fff;background:#c0392b;margin-top:6px;cursor:pointer}</style></head>"
"<body><h1>ESP32-P4 Car</h1><div class='s' id='st'>就绪</div>"
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
"<script>var DI=document.getElementById('di'),SP=document.getElementById('sp');"
"function go(c){var u='/ctrl?cmd='+c+'&dist='+DI.value+'&speed='+SP.value;"
"fetch(u).then(function(r){return r.text()}).then(function(t){document.getElementById('st').textContent=t})}"
"DI.oninput=function(){document.getElementById('d2').textContent=DI.value+'cm'};"
"SP.oninput=function(){document.getElementById('s2').textContent=SP.value};</script></body></html>";

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

/* ==================== Public API ==================== */
esp_err_t web_control_start(motor_control_cb_t callback)
{
    g_motor_callback = callback;
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 12288;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) { ESP_LOGE(TAG, "HTTP start fail"); return ESP_FAIL; }
    httpd_uri_t u = { .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    u.uri = "/";   httpd_register_uri_handler(server, &u);
    u.uri = "/ctrl"; u.handler = ctrl_handler; httpd_register_uri_handler(server, &u);
    ESP_LOGI(TAG, "Web control ready. Open http://192.168.4.1/");
    return ESP_OK;
}
