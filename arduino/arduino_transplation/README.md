# ESP32-P4 IMX219 摄像头 Arduino 库

这是一个专为ESP32-P4设计的IMX219摄像头驱动库，可以直接在Arduino IDE中使用。

## 功能特性

- 支持IMX219 MIPI CSI摄像头
- 输出分辨率：1536x1232 @ 30fps (RAW10格式)
- 内置硬件JPEG编码器支持
- 提供图像裁剪、缩放、去马赛克、灰度转换等功能
- 示例程序可以直接输出96x96灰度图像到串口

## 硬件要求

- ESP32-P4开发板
- IMX219摄像头模块 (8MP，MIPI接口)
- 接线说明：
  - I2C SCL: GPIO 8
  - I2C SDA: GPIO 7
  - XCLK: GPIO 20
  - MIPI CSI接口：按开发板说明连接

## 安装方法

1. 下载本库
2. 将整个 `arduino_transplation` 文件夹复制到Arduino的libraries目录
3. 重启Arduino IDE
4. 在文件->示例->ESP32_P4_IMX219中可以找到示例程序

重要说明：

- 如果你需要在 Arduino IDE 里“出图”（MIPI CSI 采集，依赖 `esp_video/esp_cam_sensor`），需要先制作带这些组件的自定义 Arduino Core。
- 参考本仓库文档：`arduino/CORE_REBUILD.md`

## 使用方法

### 基础使用

```cpp
#include <imx219.h>

void setup() {
    // 初始化IMX219传感器
    // 参数：I2C编号, SDA引脚, SCL引脚, I2C频率
    imx219_init(0, 7, 8, 100000);
    
    // 启动流
    imx219_set_stream(true);
}

void loop() {
    // 你的代码
}
```

### 高级功能

- 设置增益：`imx219_i2c_set_gain(0xE8);` (范围：0x00-0xFF)
- 设置曝光：`imx219_i2c_set_exposure(1750);` (单位：行)
- 写入寄存器：`imx219_write_reg(reg_addr, value);`
- 读取寄存器：`imx219_read_reg(reg_addr, &value);`

## 示例程序

### IMX219_Camera_Example

这个示例程序演示了如何捕获摄像头数据，处理后通过串口输出96x96灰度图像。

输出格式：
- 同步头：3字节 (0xAA 0x55 0xAA)
- 图像数据：96x96 = 9216字节灰度数据
- 串口波特率：921600

## 注意事项

1. 本库仅支持ESP32-P4芯片，不支持其他ESP32系列芯片
2. 需要使用支持ESP32-P4的Arduino核心版本 (>=3.0.0)
3. 建议启用PSRAM以获得更好的性能和更大的缓冲区
4. CSI接口配置需要根据具体硬件进行调整

## 许可证

MIT License
