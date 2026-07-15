/*
 * 人脸识别模块 — 基于 ESP-DL MobileFaceNet 特征提取
 *
 * 功能:
 *   - 录入人脸: 从 RGB565 帧 + 人脸框提取 2048 维特征，存入 SPIFFS 数据库
 *   - 识别人脸: 对检测到的人脸提取特征，与数据库逐一匹配余弦相似度
 *   - 数据库管理: 增删查 CSV (SPIFFS: id,name,feat_hash)
 *
 * 注意:
 *   - MobileFaceNet 只能识别人脸正视图 (±30 度)
 *   - 侧脸不要尝试录入
 *   - 识别结果仅作为辅助确认，不是主要检测手段
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FR_MAX_FACES        8
#define FR_MAX_NAME_LEN     32

/* ── 匹配结果 ── */
typedef struct {
    uint16_t id;
    float    similarity;
} fr_match_t;

typedef struct {
    fr_match_t matches[FR_MAX_FACES];
    uint8_t    count;
    bool       recognized;
} fr_recog_result_t;

/* ═══════════════════════════════════════════════════════════ *
 *  公开 API
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化人脸识别: 挂载 SPIFFS, 加载 MobileFaceNet 模型, 打开/创建 DB。
 * @param partition_label  SPIFFS 分区标签 (例如 "storage")
 * @param mount_point      挂载点 (例如 "/spiffs")
 * @param db_filename      DB 文件名 (例如 "/spiffs/face.db")
 */
esp_err_t face_recognition_init(const char *partition_label,
                                 const char *mount_point,
                                 const char *db_filename);

/** 启用/禁用识别 (异步任务) */
void face_recognition_set_enabled(bool en);
bool face_recognition_is_enabled(void);

/**
 * @brief 录入人脸: 传入 RGB565 快照 + 检测框，提取特征并存入 DB。
 * @param name    可读名称 (自动分配 ID)
 * @param box     检测框 [x, y, w, h] (原始分辨率坐标)
 * @param out_id  出参: 分配的 DB ID (失败=0)
 * @return ESP_OK 成功。
 */
esp_err_t face_recognition_enroll(const uint8_t *rgb565, int w, int h, int stride,
                                   const char *name, const int box[4],
                                   uint16_t *out_id);

/**
 * @brief 异步识别 (非阻塞): 对 det_boxes 中的每个人脸逐一识别。
 *   在 RecognTask 中调用，结果存内部缓冲区供 /status 读取。
 *   此操作耗时 ~200ms/脸，不要在 pre-JPEG 回调中调用！
 */
void face_recognition_recognize_async(const uint8_t *rgb565, int w, int h, int stride,
                                       const int boxes[][4], const float *scores,
                                       int box_count);

/** 获取最近一次识别结果 (线程安全) */
fr_recog_result_t face_recognition_get_latest(void);

/** 获取数据库人脸数 */
int face_recognition_count(void);

/** 列出所有已注册人脸 */
int face_recognition_list(uint16_t *ids, char names[][FR_MAX_NAME_LEN], int max_count);

/** 按 ID 删除人脸 */
esp_err_t face_recognition_delete(uint16_t id);

/** 清空数据库 */
esp_err_t face_recognition_clear(void);

#ifdef __cplusplus
}
#endif
