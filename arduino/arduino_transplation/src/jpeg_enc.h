#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化JPEG硬件编码器
 *
 * @param width 图像宽度
 * @param height 图像高度
 * @return esp_err_t 错误码
 */
esp_err_t jpeg_enc_init(int width, int height);

/**
 * @brief 编码图像为JPEG格式
 *
 * @param src_data 原始图像数据
 * @param src_size 原始数据大小
 * @param width 图像宽度
 * @param height 图像高度
 * @param pixel_format 像素格式 (V4L2_PIX_FMT_*)
 * @param out_data 输出JPEG数据缓冲区 (需要调用jpeg_enc_free释放)
 * @param out_size 输出JPEG数据大小
 * @return esp_err_t 错误码
 */
esp_err_t jpeg_enc_process(const void *src_data, int src_size, int width, int height,
                           uint32_t pixel_format, uint8_t **out_data, int *out_size);

/**
 * @brief 释放JPEG编码缓冲区
 *
 * @param data 要释放的缓冲区指针
 */
void jpeg_enc_free(uint8_t *data);

#ifdef __cplusplus
}
#endif
