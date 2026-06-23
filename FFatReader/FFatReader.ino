#include <Arduino.h>
#include <FFat.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

extern "C" {
#include "esp_partition.h"
#include "wear_levelling.h"
}

#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
#endif

#include "USB.h"
#include "USBMSC.h"

static constexpr uint32_t kDebugBaud = 921600;

static constexpr int kImageWidth = 96;
static constexpr int kImageHeight = 96;
static constexpr size_t kImageBytes = (size_t)kImageWidth * (size_t)kImageHeight;

static constexpr const char *kMountPoint = "/ffat";
static constexpr bool kFormatOnFail = true;
static constexpr const char *kPartitionLabel = "ffat";

enum class InterfaceMode : uint8_t { Serial = 0, UsbMsc = 1 };
//change here to change the mass storage mode
static constexpr InterfaceMode kInterfaceMode = InterfaceMode::UsbMsc;
static InterfaceMode s_interface_mode = kInterfaceMode;

static char s_run_dir[32] = {0};
static char s_last_file[96] = {0};

static wl_handle_t s_wl = WL_INVALID_HANDLE;
static USBMSC s_msc;
static uint32_t s_msc_block_count = 0;
static uint16_t s_msc_block_size = 0;

static void print_help() {
  Serial.println("Commands:");
  Serial.println("  help");
  if (s_interface_mode == InterfaceMode::UsbMsc) {
    Serial.println("MSC mode: use host file manager to copy/delete files");
    return;
  }
  Serial.println("  gen_template");
  Serial.println("  ls [path]");
  Serial.println("  get <path>");
  Serial.println("  get_last");
  Serial.println("  bundle_runs");
  Serial.println("  bundle <path>");
  Serial.println("  clean_runs");
  Serial.println("  clean <path>");
  Serial.println("  clean_all");
}

static void list_dir(const char *path) {
  DIR *dir = opendir(path);
  if (!dir) {
    Serial.println("ERR open dir");
    return;
  }
  for (;;) {
    struct dirent *ent = readdir(dir);
    if (!ent) {
      break;
    }
    const char *name = ent->d_name;
    if (!name || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
      continue;
    }
    char full[192];
    snprintf(full, sizeof(full), "%s/%s", path, name);
    struct stat st;
    if (stat(full, &st) == 0) {
      if (S_ISDIR(st.st_mode)) {
        Serial.printf("D %s\n", full);
      } else {
        Serial.printf("F %s %lu\n", full, (unsigned long)st.st_size);
      }
    } else {
      Serial.printf("? %s\n", full);
    }
  }
  closedir(dir);
}

static bool remove_recursive(const char *path) {
  struct stat st;
  if (stat(path, &st) != 0) {
    return false;
  }
  if (S_ISDIR(st.st_mode)) {
    DIR *dir = opendir(path);
    if (!dir) {
      return false;
    }
    for (;;) {
      struct dirent *ent = readdir(dir);
      if (!ent) {
        break;
      }
      const char *name = ent->d_name;
      if (!name || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
        continue;
      }
      char child[256];
      snprintf(child, sizeof(child), "%s/%s", path, name);
      remove_recursive(child);
    }
    closedir(dir);
    return rmdir(path) == 0;
  }
  return unlink(path) == 0;
}

static void clean_runs() {
  DIR *dir = opendir(kMountPoint);
  if (!dir) {
    Serial.println("ERR open dir");
    return;
  }
  for (;;) {
    struct dirent *ent = readdir(dir);
    if (!ent) {
      break;
    }
    const char *name = ent->d_name;
    if (!name) {
      continue;
    }
    if (strncmp(name, "run_", 4) != 0) {
      continue;
    }
    char full[192];
    snprintf(full, sizeof(full), "%s/%s", kMountPoint, name);
    if (remove_recursive(full)) {
      Serial.printf("OK %s\n", full);
    } else {
      Serial.printf("ERR %s\n", full);
    }
  }
  closedir(dir);
}

static void clean_all() {
  DIR *dir = opendir(kMountPoint);
  if (!dir) {
    Serial.println("ERR open dir");
    return;
  }
  for (;;) {
    struct dirent *ent = readdir(dir);
    if (!ent) {
      break;
    }
    const char *name = ent->d_name;
    if (!name || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
      continue;
    }
    char full[192];
    snprintf(full, sizeof(full), "%s/%s", kMountPoint, name);
    if (remove_recursive(full)) {
      Serial.printf("OK %s\n", full);
    } else {
      Serial.printf("ERR %s\n", full);
    }
  }
  closedir(dir);
}

static size_t bundle_count_files_rec(const char *root, bool only_run_dirs, bool at_root) {
  DIR *dir = opendir(root);
  if (!dir) {
    return 0;
  }
  size_t total = 0;
  for (;;) {
    struct dirent *ent = readdir(dir);
    if (!ent) {
      break;
    }
    const char *name = ent->d_name;
    if (!name || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
      continue;
    }
    if (only_run_dirs && at_root && strncmp(name, "run_", 4) != 0) {
      continue;
    }
    char full[256];
    snprintf(full, sizeof(full), "%s/%s", root, name);
    struct stat st;
    if (stat(full, &st) != 0) {
      continue;
    }
    if (S_ISDIR(st.st_mode)) {
      total += bundle_count_files_rec(full, false, false);
    } else {
      total++;
    }
  }
  closedir(dir);
  return total;
}

static void bundle_send_files_rec(const char *root, const char *bundle_root, bool only_run_dirs, bool at_root) {
  DIR *dir = opendir(root);
  if (!dir) {
    return;
  }
  for (;;) {
    struct dirent *ent = readdir(dir);
    if (!ent) {
      break;
    }
    const char *name = ent->d_name;
    if (!name || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
      continue;
    }
    if (only_run_dirs && at_root && strncmp(name, "run_", 4) != 0) {
      continue;
    }
    char full[256];
    snprintf(full, sizeof(full), "%s/%s", root, name);
    struct stat st;
    if (stat(full, &st) != 0) {
      continue;
    }
    if (S_ISDIR(st.st_mode)) {
      bundle_send_files_rec(full, bundle_root, false, false);
      continue;
    }
    const size_t base_len = strlen(bundle_root);
    const char *rel = full;
    if (strncmp(full, bundle_root, base_len) == 0 && full[base_len] == '/') {
      rel = full + base_len + 1;
    }
    Serial.printf("FILE %s %lu\n", rel, (unsigned long)st.st_size);
    FILE *f = fopen(full, "rb");
    if (f) {
      emit_file_payload_base64(f);
      fclose(f);
    }
    Serial.println("ENDFILE");
  }
  closedir(dir);
}

static void bundle_dir(const char *root, bool only_run_dirs) {
  struct stat st;
  if (stat(root, &st) != 0 || !S_ISDIR(st.st_mode)) {
    Serial.println("ERR open dir");
    return;
  }
  size_t files = bundle_count_files_rec(root, only_run_dirs, true);
  Serial.printf("BEGIN_BUNDLE %s %u\n", root, (unsigned)files);
  bundle_send_files_rec(root, root, only_run_dirs, true);
  Serial.println("END_BUNDLE");
}

static void b64_emit_triplet(uint8_t a, uint8_t b, uint8_t c, int pad, int *col) {
  static const char *kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint32_t v = ((uint32_t)a << 16) | ((uint32_t)b << 8) | (uint32_t)c;
  char out[4];
  out[0] = kB64[(v >> 18) & 0x3F];
  out[1] = kB64[(v >> 12) & 0x3F];
  out[2] = (pad >= 2) ? '=' : kB64[(v >> 6) & 0x3F];
  out[3] = (pad >= 1) ? '=' : kB64[v & 0x3F];
  for (int i = 0; i < 4; i++) {
    Serial.write(out[i]);
    (*col)++;
    if (*col >= 76) {
      Serial.write('\n');
      *col = 0;
    }
  }
}

static bool emit_file_payload_base64(FILE *f) {
  uint8_t buf[3];
  int col = 0;
  while (true) {
    size_t n = fread(buf, 1, 3, f);
    if (n == 3) {
      b64_emit_triplet(buf[0], buf[1], buf[2], 0, &col);
    } else if (n == 2) {
      b64_emit_triplet(buf[0], buf[1], 0, 1, &col);
      break;
    } else if (n == 1) {
      b64_emit_triplet(buf[0], 0, 0, 2, &col);
      break;
    } else {
      break;
    }
  }
  if (col != 0) {
    Serial.write('\n');
  }
  return true;
}

static void send_file_base64(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    Serial.println("ERR open file");
    return;
  }

  if (fseek(f, 0, SEEK_END) != 0) {
    fclose(f);
    Serial.println("ERR seek");
    return;
  }
  long size = ftell(f);
  if (size < 0) {
    fclose(f);
    Serial.println("ERR size");
    return;
  }
  if (fseek(f, 0, SEEK_SET) != 0) {
    fclose(f);
    Serial.println("ERR seek");
    return;
  }

  Serial.printf("BEGIN %s %ld\n", path, size);
  emit_file_payload_base64(f);
  Serial.println("END");
  fclose(f);
}

static bool read_line(char *out, size_t cap) {
  static size_t pos = 0;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch < 0) {
      break;
    }
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      out[pos] = 0;
      pos = 0;
      return true;
    }
    if (pos + 1 < cap) {
      out[pos++] = (char)ch;
    }
  }
  return false;
}

static bool mount_ffat() {
  if (FFat.begin(false, kMountPoint, 5, "ffat")) {
    return true;
  }
  if (kFormatOnFail && FFat.begin(true, kMountPoint, 5, "ffat")) {
    return true;
  }
  return false;
}

static int32_t msc_on_read(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  if (s_wl == WL_INVALID_HANDLE) {
    return -1;
  }
  const size_t addr = (size_t)lba * (size_t)s_msc_block_size + (size_t)offset;
  if (wl_read(s_wl, addr, buffer, bufsize) != ESP_OK) {
    return -1;
  }
  return (int32_t)bufsize;
}

static int32_t msc_on_write(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  (void)lba;
  (void)offset;
  (void)buffer;
  (void)bufsize;
  return -1;
}

static bool start_msc_ffat() {
  const esp_partition_t *part =
    esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, kPartitionLabel);
  if (!part) {
    Serial.println("ERR no ffat partition");
    return false;
  }
  if (wl_mount(part, &s_wl) != ESP_OK) {
    Serial.println("ERR wl_mount");
    s_wl = WL_INVALID_HANDLE;
    return false;
  }

  const size_t size = wl_size(s_wl);
  s_msc_block_size = (uint16_t)CONFIG_WL_SECTOR_SIZE;
  s_msc_block_count = (uint32_t)(size / (size_t)s_msc_block_size);

  s_msc.vendorID("ESP32P4");
  s_msc.productID("FFat");
  s_msc.productRevision("1.0");
  s_msc.onRead(msc_on_read);
  s_msc.onWrite(msc_on_write);
  s_msc.mediaPresent(true);
  s_msc.isWritable(false);
  if (!s_msc.begin(s_msc_block_count, s_msc_block_size)) {
    Serial.println("ERR MSC begin");
    wl_unmount(s_wl);
    s_wl = WL_INVALID_HANDLE;
    return false;
  }
  USB.begin();
  Serial.printf("MSC started: blocks=%lu block_size=%u\n", (unsigned long)s_msc_block_count, (unsigned)s_msc_block_size);
  return true;
}

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

static void gen_template() {
  if (!make_run_dir()) {
    Serial.println("Create run dir failed");
    return;
  }
  Serial.printf("Run dir: %s\n", s_run_dir);

  static uint8_t img[kImageBytes];
  for (int y = 0; y < kImageHeight; y++) {
    for (int x = 0; x < kImageWidth; x++) {
      img[(y * kImageWidth) + x] = (uint8_t)(((y * 255) / (kImageHeight - 1)) & 0xFF);
    }
  }

  char path[96];
  snprintf(path, sizeof(path), "%s/test.pgm", s_run_dir);
  bool ok = write_pgm(path, img);
  if (ok) {
    snprintf(s_last_file, sizeof(s_last_file), "%s", path);
  }
  Serial.printf("Write %s %s\n", path, ok ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(kDebugBaud);
  delay(200);
  Serial.println("P4 FFatReader start");

  if (s_interface_mode == InterfaceMode::UsbMsc) {
#if ARDUINO_USB_MODE == 1
    Serial.println("ERR USB Mode must be USB-OTG (TinyUSB)");
    s_interface_mode = InterfaceMode::Serial;
#else
    if (!start_msc_ffat()) {
      Serial.println("MSC start failed");
    } else {
      Serial.println("MSC mode: use your computer to copy/delete files");
    }
    print_help();
    return;
#endif
  }

  if (!mount_ffat()) {
    Serial.println("FFat mount failed");
    return;
  }

  Serial.printf("Mounted FFat: total=%lluKB used=%lluKB\n",
                (unsigned long long)(FFat.totalBytes() / 1024),
                (unsigned long long)(FFat.usedBytes() / 1024));

  print_help();
}

void loop() {
  static char line[160];
  if (!read_line(line, sizeof(line))) {
    delay(5);
    return;
  }

  if (s_interface_mode == InterfaceMode::UsbMsc) {
    if (strcmp(line, "help") == 0) {
      print_help();
    } else {
      Serial.println("ERR MSC mode: use host file manager");
    }
    return;
  }

  if (strcmp(line, "help") == 0) {
    print_help();
    return;
  }

  if (strcmp(line, "gen_template") == 0) {
    gen_template();
    return;
  }

  if (strcmp(line, "get_last") == 0) {
    if (s_last_file[0]) {
      send_file_base64(s_last_file);
    } else {
      Serial.println("ERR no last file");
    }
    return;
  }

  if (strncmp(line, "ls", 2) == 0) {
    const char *p = line + 2;
    while (*p == ' ') {
      p++;
    }
    if (*p == 0) {
      list_dir(kMountPoint);
    } else {
      list_dir(p);
    }
    return;
  }

  if (strncmp(line, "get ", 4) == 0) {
    const char *p = line + 4;
    while (*p == ' ') {
      p++;
    }
    if (*p == 0) {
      Serial.println("ERR missing path");
      return;
    }
    send_file_base64(p);
    return;
  }

  if (strcmp(line, "bundle_runs") == 0) {
    bundle_dir(kMountPoint, true);
    return;
  }

  if (strncmp(line, "bundle ", 7) == 0) {
    const char *p = line + 7;
    while (*p == ' ') {
      p++;
    }
    if (*p == 0) {
      Serial.println("ERR missing path");
      return;
    }
    bundle_dir(p, false);
    return;
  }

  if (strcmp(line, "clean_runs") == 0) {
    clean_runs();
    return;
  }

  if (strcmp(line, "clean_all") == 0) {
    clean_all();
    return;
  }

  if (strncmp(line, "clean ", 6) == 0) {
    const char *p = line + 6;
    while (*p == ' ') {
      p++;
    }
    if (*p == 0) {
      Serial.println("ERR missing path");
      return;
    }
    if (remove_recursive(p)) {
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
    return;
  }

  Serial.println("ERR unknown command");
}
