#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include <Arduino.h>
#include <Wire.h>

#define IMX219_I2C_ADDR 0x10
#define IMX219_PID 0x0219

#ifdef __cplusplus
extern "C" {
#endif

// IMX219 驱动初始化
esp_err_t imx219_init(uint8_t i2c_num, uint8_t sda_pin, uint8_t scl_pin, uint32_t freq);

// 写入寄存器
esp_err_t imx219_write_reg(uint16_t reg, uint8_t val);

// 读取寄存器
esp_err_t imx219_read_reg(uint16_t reg, uint8_t *val);

// 设置增益 (0x00-0xFF)
esp_err_t imx219_set_gain(uint8_t gain);

// 设置曝光 (行数)
esp_err_t imx219_set_exposure(uint16_t exposure);

// 启动/停止流
esp_err_t imx219_set_stream(bool enable);

// 初始化传感器配置（1536x1232 30fps RAW10）
esp_err_t imx219_set_default_config();

#ifdef __cplusplus
}
#endif
