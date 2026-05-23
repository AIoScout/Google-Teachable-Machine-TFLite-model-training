/*
 * ESP32-P4 IMX219 Camera Example for Arduino
 * 功能：捕获IMX219摄像头数据，转换为灰度图像，通过串口输出
 * 输出格式：0xAA 0x55 0xAA + 96x96灰度数据 (9216字节)
 * 串口波特率：921600
 */

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "imx219.h"
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE 1
#ifdef CONFIG_ESP_VIDEO_ENABLE_DVP_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_DVP_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_DVP_VIDEO_DEVICE 0
#ifdef CONFIG_ESP_VIDEO_ENABLE_SPI_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_SPI_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_SPI_VIDEO_DEVICE 0
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#include "esp_cam_ctlr_csi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "jpeg_enc.h"

#if defined(__riscv)
#include "riscv/rv_utils.h"
#endif

extern "C" {
struct esp_video;
esp_err_t esp_video_vfs_dev_register(const char *name, struct esp_video *video);
struct esp_video *esp_video_device_get_object(const char *name);
void imx219_force_link(void);
}

static const char *TAG = "app_main";

extern "C" esp_err_t __real_esp_cam_new_csi_ctlr(const esp_cam_ctlr_csi_config_t *config, esp_cam_ctlr_handle_t *ret_handle);
extern "C" esp_err_t __wrap_esp_cam_new_csi_ctlr(const esp_cam_ctlr_csi_config_t *config, esp_cam_ctlr_handle_t *ret_handle) {
    esp_cam_ctlr_csi_config_t cfg = *config;
    if (cfg.clk_src != MIPI_CSI_PHY_CLK_SRC_PLL_F20M &&
        cfg.clk_src != MIPI_CSI_PHY_CLK_SRC_PLL_F25M &&
        cfg.clk_src != MIPI_CSI_PHY_CLK_SRC_RC_FAST) {
        cfg.clk_src = MIPI_CSI_PHY_CLK_SRC_PLL_F20M;
    }
    return __real_esp_cam_new_csi_ctlr(&cfg, ret_handle);
}

// 硬件配置
static const gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_8;
static const gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_7;
static const int I2C_MASTER_NUM = 0;
static const int I2C_MASTER_FREQ_HZ = 100000;
static const gpio_num_t XCLK_PIN = GPIO_NUM_20;
static const int XCLK_FREQ_HZ = 24000000;

// 图像配置
#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  96
#define OUT_HEIGHT 96

// 全局变量
static int *s_x_lut = NULL;
static int *s_y_lut = NULL;
static int fd = -1;
static void *mapped_bufs[2] = {0};
static uint8_t *rgb_buf = NULL;
static uint8_t *gray_buf = NULL;
static uint64_t last_time = 0;
static uint64_t last_frame_send = 0;
static uint32_t frame_count = 0;
static uint32_t sent_count = 0;

// 帧间隔 (微秒) - 10 FPS = 100ms
#define FRAME_INTERVAL_US 100000

// 启动XCLK时钟输出 (使用LEDC生成24MHz时钟)
static void enable_xclk(void) {
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
    ledc_timer.freq_hz = XCLK_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel;
    memset(&ledc_channel, 0, sizeof(ledc_channel));
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 1;
    ledc_channel.gpio_num = XCLK_PIN;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "XCLK Enabled on GPIO %d at %d Hz", XCLK_PIN, XCLK_FREQ_HZ);
}

// 初始化去马赛克查找表 (用于将中心区域裁剪并缩放到96x96)
static void init_demosaic_luts(int width, int height) {
    s_x_lut = (int *)malloc(OUT_WIDTH * sizeof(int));
    s_y_lut = (int *)malloc(OUT_HEIGHT * sizeof(int));

    // 中心裁剪1232x1232区域
    int crop_w = 1232;
    int crop_h = 1232;
    int x_offset = (width - crop_w) / 2;
    int y_offset = (height - crop_h) / 2;

    // 生成查找表 (确保坐标对齐到偶数，用于BGGR去马赛克)
    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = (int)(((int64_t)y * crop_h + (OUT_HEIGHT / 2)) / OUT_HEIGHT);
        s_y_lut[y] = (y_offset + src_y) & ~1;
    }
    for (int x = 0; x < OUT_WIDTH; x++) {
        int src_x = (int)(((int64_t)x * crop_w + (OUT_WIDTH / 2)) / OUT_WIDTH);
        s_x_lut[x] = (x_offset + src_x) & ~1;
    }

    ESP_LOGI(TAG, "Sensor: %dx%d, Center crop: %dx%d at offset (%d,%d), Output: %dx%d",
             width, height, crop_w, crop_h, x_offset, y_offset, OUT_WIDTH, OUT_HEIGHT);
}

// BGGR格式RAW10数据去马赛克转换为RGB888
static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height) {
    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = s_y_lut[y];
        int row0 = src_y * (width * 5 / 4);  // RAW10格式: 4个像素占5字节
        int row1 = (src_y + 1) * (width * 5 / 4);
        int out_row = y * OUT_WIDTH;
        for (int x = 0; x < OUT_WIDTH; x++) {
            int src_x = s_x_lut[x];
            int col = (src_x >> 2) * 5 + (src_x % 4);
            // BGGR格式:
            // 行偶: B G B G ...
            // 行奇: G R G R ...
            uint8_t b = raw10[row0 + col];
            uint8_t g = (raw10[row0 + col + 1] + raw10[row1 + col]) >> 1;
            uint8_t r = raw10[row1 + col + 1];
            int out_idx = (out_row + x) * 3;
            rgb[out_idx + 0] = r;
            rgb[out_idx + 1] = g;
            rgb[out_idx + 2] = b;
        }
    }
}

// RGB888转换为灰度图像
static void rgb_to_gray(const uint8_t *rgb, uint8_t *gray, int pixel_count) {
    for (int i = 0; i < pixel_count; i++) {
        int idx = i * 3;
        uint8_t r = rgb[idx + 0];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];
        // 灰度转换公式: Y = 0.3R + 0.59G + 0.11B
        gray[i] = (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
    }
}

// 同步头
const uint8_t syncHeader[] = {0xAA, 0x55, 0xAA};

#if defined(__riscv)
static uint32_t read_mstatus(void) {
    uint32_t v;
    asm volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}

static void fpu_enable_task(void *arg) {
    rv_utils_enable_fpu();
    ESP_LOGI(TAG, "FPU enable core=%d mstatus=0x%08x", (int)xPortGetCoreID(), (unsigned)read_mstatus());
    xTaskNotifyGive((TaskHandle_t)arg);
    vTaskDelete(NULL);
}

static void enable_fpu_all_cores(void) {
    rv_utils_enable_fpu();
    ESP_LOGI(TAG, "FPU enable core=%d mstatus=0x%08x", (int)xPortGetCoreID(), (unsigned)read_mstatus());
#if defined(configNUMBER_OF_CORES) && (configNUMBER_OF_CORES > 1)
    const BaseType_t self_core = xPortGetCoreID();
    const BaseType_t other_core = self_core ? 0 : 1;
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    if (xTaskCreatePinnedToCore(fpu_enable_task, "fpu_en", 2048, self, configMAX_PRIORITIES - 1, NULL, other_core) == pdPASS) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
#endif
}
#endif

void setup() {
    // 初始化串口
    Serial.begin(921600);
    delay(100);
    Serial.println("ESP32-P4 IMX219 Camera Example");

#if defined(__riscv)
    enable_fpu_all_cores();
#endif

    // 初始化NVS (用于摄像头驱动)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 初始化查找表
    init_demosaic_luts(IMG_WIDTH, IMG_HEIGHT);

    // 初始化JPEG编码器
    jpeg_enc_init(OUT_WIDTH, OUT_HEIGHT);

    // 启动XCLK时钟
    enable_xclk();
    delay(100);

  imx219_force_link();

    // 初始化CSI接口
   esp_video_init_csi_config_t csi_config;
memset(&csi_config, 0, sizeof(csi_config));
csi_config.sccb_config.init_sccb = true;
csi_config.sccb_config.i2c_config.port = I2C_MASTER_NUM;
csi_config.sccb_config.i2c_config.scl_pin = I2C_MASTER_SCL_IO;
csi_config.sccb_config.i2c_config.sda_pin = I2C_MASTER_SDA_IO;
csi_config.sccb_config.freq = I2C_MASTER_FREQ_HZ;
csi_config.reset_pin = GPIO_NUM_NC;
csi_config.pwdn_pin = GPIO_NUM_NC;

esp_video_init_config_t cam_config;
memset(&cam_config, 0, sizeof(cam_config));
cam_config.csi = &csi_config;

err = esp_video_init(&cam_config);
if (err != ESP_OK) {
  ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(err));
  Serial.println("esp_video_init failed!");
  return;
}

    // 打开CSI设备
    fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open CSI device: %s", strerror(errno));
        Serial.println("Failed to open CSI device!");
        return;
    }

    // 设置图像格式 (1536x1232 RAW10)
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_WIDTH;
    fmt.fmt.pix.height = IMG_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
    ioctl(fd, VIDIOC_S_FMT, &fmt);

    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(fd, VIDIOC_REQBUFS, &req);

    // 映射缓冲区
    for (int i = 0; i < 2; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof(b));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        mapped_bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    // 启动流
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);

    // 分配图像缓冲区 (使用PSRAM)
    rgb_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT * 3, MALLOC_CAP_SPIRAM);
    gray_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT, MALLOC_CAP_SPIRAM);

    last_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Camera setup complete. Sending 96x96 grayscale images...");
    Serial.println("Camera setup complete. Sending 96x96 grayscale images...");
    ESP_LOGI(TAG, "rgb_buf=%p, gray_buf=%p", rgb_buf, gray_buf);
}

void loop() {
    if (fd < 0 || rgb_buf == NULL || gray_buf == NULL) {
        delay(1000);
        Serial.println("Camera not initialized!");
        return;
    }

    // 每秒打印帧率信息
    uint64_t now = esp_timer_get_time();
    if ((now - last_time) >= 1000000) {
        ESP_LOGI(TAG, "Capture FPS: %lu, Send FPS: %lu",
                 (unsigned long)frame_count, (unsigned long)sent_count);
        Serial.printf("FPS: Capture=%lu, Send=%lu\n", frame_count, sent_count);
        frame_count = 0;
        sent_count = 0;
        last_time = now;
    }

    // 捕获帧
    struct v4l2_buffer buf_dq;
    memset(&buf_dq, 0, sizeof(buf_dq));
    buf_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_dq.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_DQBUF, &buf_dq) == 0) {
        frame_count++;
        uint8_t *raw_data = (uint8_t *)mapped_bufs[buf_dq.index];

        // 按设定的帧率发送
        if (now - last_frame_send >= FRAME_INTERVAL_US) {
            // 去马赛克转换为RGB
            demosaic_bggr_to_rgb(raw_data, rgb_buf, IMG_WIDTH, IMG_HEIGHT);
            // 转换为灰度
            rgb_to_gray(rgb_buf, gray_buf, OUT_WIDTH * OUT_HEIGHT);

            // 临时关闭日志，避免干扰串口输出
            esp_log_level_t prev_level = esp_log_level_get("*");
            esp_log_level_set("*", ESP_LOG_NONE);

            // 输出同步头和灰度数据
            Serial.write(syncHeader, sizeof(syncHeader));
            Serial.write(gray_buf, OUT_WIDTH * OUT_HEIGHT);
            Serial.flush();

            // 恢复日志级别
            esp_log_level_set("*", prev_level);

            last_frame_send = now;
            sent_count++;
        }

        // 归还缓冲区
        ioctl(fd, VIDIOC_QBUF, &buf_dq);
    }

    delay(1);
}
