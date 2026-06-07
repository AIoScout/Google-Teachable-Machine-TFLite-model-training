#include <Arduino.h>
#include <SD_MMC.h>
#include <driver/gpio.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

// SDReader: minimal SD card test for ESP32-P4.
// It mounts the on-board MicroSD slot, creates a new /run_XXXX folder, and
// writes a small 96x96 grayscale PGM image.
//
// Notes:
// - The SD card should be formatted as FAT32. exFAT is not supported by default.
// - For on-board slots, you usually do NOT need SD_MMC.setPins().

static constexpr uint32_t kDebugBaud = 921600;

static constexpr int kImageWidth = 96;
static constexpr int kImageHeight = 96;
static constexpr size_t kImageBytes = (size_t)kImageWidth * (size_t)kImageHeight;

static constexpr const char *kMountPoint = "/sdcard";

#ifndef BOARD_SDMMC_POWER_PIN
#define BOARD_SDMMC_POWER_PIN 45
#endif

#ifndef BOARD_SDMMC_POWER_ON_LEVEL
#define BOARD_SDMMC_POWER_ON_LEVEL 0
#endif

static void sdcard_power_cycle(int on_level) {
  gpio_set_direction((gpio_num_t)BOARD_SDMMC_POWER_PIN, GPIO_MODE_OUTPUT);
  int off_level = on_level ? 0 : 1;
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, off_level);
  delay(200);
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, on_level);
  delay(300);
}

// Set this to true only if you are wiring SD_MMC manually (custom pins).
static constexpr bool kUseSdMmcCustomPins = false;
static constexpr int kSdClkPin = -1;
static constexpr int kSdCmdPin = -1;
static constexpr int kSdD0Pin = -1;
static constexpr int kSdD1Pin = -1;
static constexpr int kSdD2Pin = -1;
static constexpr int kSdD3Pin = -1;
static char s_run_dir[32] = {0};
static bool s_using_sd_mmc = false;

static bool mount_sd() {
  s_using_sd_mmc = false;

  // FireBeetle 2 ESP32-P4 TF slot uses SDMMC slot0 IOMUX pins:
  // CLK=GPIO43, CMD=GPIO44, D0=GPIO39, D1=GPIO40, D2=GPIO41, D3=GPIO42, EN=GPIO45.
  //
  // CMD/D0/D1/D2/D3 need pull-ups. Some boards rely on internal pull-ups,
  // so we enable them here to improve compatibility.
  gpio_set_pull_mode((gpio_num_t)44, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)39, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)40, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)41, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)42, GPIO_PULLUP_ONLY);

  #ifdef SOC_SDMMC_IO_POWER_EXTERNAL
  SD_MMC.setPowerChannel(-1);
  #endif

  const int on_levels[] = {BOARD_SDMMC_POWER_ON_LEVEL, BOARD_SDMMC_POWER_ON_LEVEL ? 0 : 1};
  for (int attempt = 0; attempt < 12; attempt++) {
    for (size_t lv = 0; lv < (sizeof(on_levels) / sizeof(on_levels[0])); lv++) {
      sdcard_power_cycle(on_levels[lv]);

      SD_MMC.end();
      if (SD_MMC.begin(kMountPoint, false, false, SDMMC_FREQ_DEFAULT)) {
        s_using_sd_mmc = true;
        return true;
      }

      SD_MMC.end();
      if (SD_MMC.begin(kMountPoint, false, false, 400)) {
        s_using_sd_mmc = true;
        return true;
      }
    }
    delay(200);
  }

  s_using_sd_mmc = false;
  SD_MMC.end();
  return false;
}


// Write a 96x96 grayscale image as a binary PGM (P5) file.
static bool write_pgm(const char *path, const uint8_t *gray) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    return false;
  }
  fprintf(f, "P5\n%d %d\n255\n", kImageWidth, kImageHeight);
  size_t written = fwrite(gray, 1, kImageBytes, f);
  fclose(f);
  return written == kImageBytes;
}

// Create a new folder "/run_XXXX" by scanning the SD root directory and using
// (max existing id + 1). No NVS is used, so deleting folders on the SD card
// naturally changes the next run number.
static bool make_run_dir() {
  uint32_t max_id = 0;

  DIR *dir = opendir(kMountPoint);
  if (dir) {
    for (;;) {
      struct dirent *ent = readdir(dir);
      if (!ent) {
        break;
      }
      const char *p = ent->d_name;
      if (strncmp(p, "run_", 4) != 0) {
        continue;
      }
      uint32_t id = atoi(p + 4);
      if (id && id <= 9999 && id > max_id) {
        max_id = id;
      }
    }
    closedir(dir);
  }

  for (uint32_t i = 0; i < 1000; i++) {
    uint32_t id = max_id + 1 + i;
    snprintf(s_run_dir, sizeof(s_run_dir), "%s/run_%04d", kMountPoint, id);
    if (mkdir(s_run_dir, 0775) == 0) {
      return true;
    }
  }

  return false;
}

void setup() {
  Serial.begin(kDebugBaud);
  delay(200);
  Serial.println("P4 SDReader start");

  if (kUseSdMmcCustomPins) {
    if (!SD_MMC.setPins(kSdClkPin, kSdCmdPin, kSdD0Pin, kSdD1Pin, kSdD2Pin, kSdD3Pin)) {
      Serial.println("SD_MMC setPins failed");
      return;
    }
  }

  if (!mount_sd()) {
    Serial.println("SD mount failed");
    return;
  }

  if (s_using_sd_mmc && SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card");
    return;
  }

  uint64_t cardSize = SD_MMC.cardSize();
  Serial.printf("SD mounted (SD_MMC), size=%lluMB\n", (unsigned long long)(cardSize / (1024 * 1024)));

  if (!make_run_dir()) {
    Serial.println("Create run dir failed");
    return;
  }
  Serial.printf("Run dir: %s\n", s_run_dir);

  // Create a simple test image (vertical grayscale gradient).
  static uint8_t img[kImageBytes];
  for (int y = 0; y < kImageHeight; y++) {
    for (int x = 0; x < kImageWidth; x++) {
      img[(y * kImageWidth) + x] = (uint8_t)(((y * 255) / (kImageHeight - 1)) & 0xFF);
    }
  }

  char path[96];
  snprintf(path, sizeof(path), "%s/test.pgm", s_run_dir);
  bool ok = write_pgm(path, img);
  Serial.printf("Write %s %s\n", path, ok ? "OK" : "FAIL");
}

void loop() {
  delay(1000);
}
