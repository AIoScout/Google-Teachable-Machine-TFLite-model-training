# Arduino IDE 出图（ESP32-P4 + IMX219）必读

你现在遇到的 `#include "esp_video_init.h"` / `esp_cam_sensor_detect.h` 找不到的问题，不是 `.ino` 写法问题，而是 **Arduino-ESP32 Core（Boards Manager 安装的 3.3.x）默认没有把** **`esp_video`** **/** **`esp_cam_sensor`** **这类 ESP-IDF 组件编进 Arduino Core 的预编译库和 include 路径**。

要在 **Arduino IDE 里** 真正跑通 ESP32-P4 + IMX219（MIPI CSI-2）“出图”，必须走 **自定义 Arduino Core**：

- 用 Espressif 官方 `esp32-arduino-lib-builder` 重新编译 `esp32p4` 的 core 静态库；
- 并把 `esp_video`、`esp_cam_sensor`（以及它们的依赖）纳入这次编译。

官方文档（核心工具与参数）：<https://docs.espressif.com/projects/arduino-esp32/en/latest/lib_builder.html>

***

## 目录约定（建议放在本仓库 arduino/ 下）

建议你把相关源码/工具都放在本仓库的 `arduino/` 下，便于管理：

- `arduino/arduino-esp32/`：Arduino-ESP32 core 源码
- `arduino/esp32-arduino-lib-builder/`：lib-builder 工具源码
- `arduino/extra_components/`：准备注入到 core 编译过程里的组件（从本仓库复用）

你可以用软链接把本仓库现有组件复用进来：

- `ESP32-P4-IMX219-PoC/managed_components/espressif__esp_video`
- `ESP32-P4-IMX219-PoC/managed_components/espressif__esp_cam_sensor`
- `ESP32-P4-IMX219-PoC/components/imx219`

***

## 步骤 1：准备 Arduino-ESP32 core 源码（匹配你当前 3.3.1）

1. clone：
   - `git clone https://github.com/espressif/arduino-esp32 arduino/arduino-esp32`
2. checkout 到与你 Arduino IDE 里一致的版本（你日志里是 3.3.1）：
   - `git checkout 3.3.1`
3. 初始化子模块：
   - `git submodule update --init --recursive`

***

## 步骤 2：准备 lib-builder

1. clone(Mac)：
   - `brew install git wget curl openssl ncurses flex bison gperf python3 cmake ninja ccache jq gsed gawk`
   - `brew install coreutils`
   - `python3 -m pip install --upgrade pip`
   - `python3 -m pip install -r tools/config_editor/requirements.txt`
   - `git clone https://github.com/espressif/esp32-arduino-lib-builder`
   - `cd esp32-arduino-lib-builder`
   - `./build.sh`

2. 选择与 Arduino-ESP32/ESP-IDF 兼容的分支/标签（建议用与 Arduino Core 对应的 release 系列）
3. 按官方文档安装依赖并执行 `./build.sh`

> 说明：lib-builder 支持直接指定 target（包含 `esp32p4`）以及把生成的库拷贝回 Arduino core 目录（`-c <path>`）。

***

## 步骤 3：把 esp\_video / esp\_cam\_sensor 纳入 core 编译

这一步是关键点：**仅运行 lib-builder 默认 build，并不会把** **`esp_video`** **这种“非 Arduino core 默认组件”编进去**。

实现方式有两种：

### 方式 A（推荐）：修改 lib-builder 的 build 工程，把 extra components 作为 IDF components 加入

你需要在 lib-builder 用于构建 IDF 静态库的工程里，增加 `EXTRA_COMPONENT_DIRS` 或者把组件放进该工程的 `components/` 目录，使其参与编译与导出 include。

由于不同 lib-builder 版本的工程目录结构不同（生成目录也会变化），这里不写死路径，按下面原则操作：

- 找到 lib-builder 里 **实际用于构建 esp32p4 静态库的 ESP-IDF 工程目录**（通常会包含 `CMakeLists.txt`、`sdkconfig`、`main/`、`components/`）
- 把以下目录加入为 components（复制或软链接）：
  - `extra_components/esp_video/`
  - `extra_components/esp_cam_sensor/`
  - `extra_components/imx219/`
- 重新运行 lib-builder 针对 `esp32p4` 的 build，并把输出 copy 回 `arduino/arduino-esp32`

### 方式 B：把 esp\_video 整体做成 Arduino Library（不推荐）

Arduino IDE 的库编译流程不理解 ESP-IDF 的 component 依赖关系、Kconfig、编译选项，`esp_video` 这种组件通常会卡在宏/链接/依赖上，维护成本非常高。

***

## 步骤 4：在 Arduino IDE 中切换到自定义 core

当你把 lib-builder 产物复制回 `arduino/arduino-esp32` 后，需要让 Arduino IDE 使用这份本地 core（而不是 Arduino15/Boards Manager 安装的 core）。

最稳的方式是把 `arduino/arduino-esp32` 放到 Arduino 的 `hardware` 搜索路径中（例如 `~/Documents/Arduino/hardware/espressif/esp32`），或用软链接指向。

***

## 最小验证用 ino

当 core 正确注入后，下列 include 必须能通过编译（不需要真正出图就能验证 include 路径已经生效）：

```cpp
#include <Arduino.h>
#include \"esp_video_init.h\"
#include \"esp_video_device.h\"
#include \"esp_video_ioctl.h\"

void setup() { Serial.begin(115200); }
void loop() {}
```

