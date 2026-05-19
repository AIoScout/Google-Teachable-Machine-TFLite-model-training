#include <string.h>

#include "imx219.h"
#include "imx219_regs.h"
#include "esp_log.h"

static const char *TAG = "imx219";
static uint8_t s_i2c_num = 0;
static bool s_initialized = false;

esp_err_t imx219_write_reg(uint16_t reg, uint8_t val) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "IMX219 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    Wire.beginTransmission(IMX219_I2C_ADDR);
    Wire.write((reg >> 8) & 0xFF);
    Wire.write(reg & 0xFF);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        ESP_LOGE(TAG, "Failed to write reg 0x%04X", reg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t imx219_read_reg(uint16_t reg, uint8_t *val) {
    if (!s_initialized || !val) {
        ESP_LOGE(TAG, "Invalid parameters or not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    Wire.beginTransmission(IMX219_I2C_ADDR);
    Wire.write((reg >> 8) & 0xFF);
    Wire.write(reg & 0xFF);
    if (Wire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Failed to send read request for reg 0x%04X", reg);
        return ESP_FAIL;
    }

    Wire.requestFrom(IMX219_I2C_ADDR, 1);
    if (Wire.available() != 1) {
        ESP_LOGE(TAG, "No data received from reg 0x%04X", reg);
        return ESP_FAIL;
    }

    *val = Wire.read();
    return ESP_OK;
}

esp_err_t imx219_set_gain(uint8_t gain) {
    return imx219_write_reg(0x0157, gain);
}

esp_err_t imx219_set_exposure(uint16_t exposure) {
    esp_err_t ret = imx219_write_reg(0x015A, (exposure >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;
    return imx219_write_reg(0x015B, exposure & 0xFF);
}

esp_err_t imx219_set_stream(bool enable) {
    ESP_LOGI(TAG, "Stream Control: %d", enable);

    esp_err_t ret = imx219_write_reg(0x0100, enable ? 0x01 : 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write stream register!");
        return ret;
    }

    delay(100);

    uint8_t reg_val = 0;
    ret = imx219_read_reg(0x0100, &reg_val);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Stream register 0x0100 read back: 0x%02X (expected: 0x%02X)",
                 reg_val, enable ? 0x01 : 0x00);
        if (reg_val != (enable ? 0x01 : 0x00)) {
            ESP_LOGE(TAG, "Stream register mismatch! Sensor may not be streaming.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read back stream register!");
    }

    return ret;
}

esp_err_t imx219_set_default_config() {
    ESP_LOGI(TAG, "Setting default IMX219 configuration (1536x1232 30fps)");

    int num_regs = sizeof(imx219_1080p_30fps) / sizeof(imx219_reg_t);
    for (int i = 0; i < num_regs; i++) {
        if (imx219_write_reg(imx219_1080p_30fps[i].reg, imx219_1080p_30fps[i].val) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write reg 0x%04X", imx219_1080p_30fps[i].reg);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "IMX219 configuration applied successfully");
    return ESP_OK;
}

esp_err_t imx219_init(uint8_t i2c_num, uint8_t sda_pin, uint8_t scl_pin, uint32_t freq) {
    ESP_LOGI(TAG, "Initializing IMX219 on I2C%d: SDA=%d, SCL=%d, Freq=%lu Hz",
             i2c_num, sda_pin, scl_pin, freq);

    if (i2c_num == 0) {
        Wire.begin(sda_pin, scl_pin, freq);
    } else {
        Wire1.begin(sda_pin, scl_pin, freq);
    }
    s_i2c_num = i2c_num;

    uint8_t id_h = 0, id_l = 0;
    if (imx219_read_reg(0x0000, &id_h) != ESP_OK ||
        imx219_read_reg(0x0001, &id_l) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor ID");
        return ESP_FAIL;
    }

    uint16_t id = (id_h << 8) | id_l;
    if (id != IMX219_PID) {
        ESP_LOGE(TAG, "ID Mismatch: Found 0x%04X, Expected 0x%04X", id, IMX219_PID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "IMX219 Detected (ID: 0x%04X)", id);

    esp_err_t ret = imx219_set_default_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply default configuration");
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "IMX219 initialized successfully");
    return ESP_OK;
}

