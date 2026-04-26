# ESP32-S3 e-Paper MP3 Player

基于 Waveshare ESP32-S3-ePaper-1.54 开发板的 MP3 播放器固件，使用 LVGL 9.2.2 绘制 e-ink UI。

![Board](https://www.waveshare.com/img/devkit/ESP32-S3-ePaper-1.54/ESP32-S3-ePaper-1.54-1.jpg)

---

## 硬件

| 模块 | 型号 |
|------|------|
| MCU | ESP32-S3 (240 MHz, 8 MB Flash, 8 MB PSRAM) |
| 屏幕 | SSD1681 200×200 B&W e-ink |
| 音频 DAC | ES8311 |
| 存储 | SD 卡（SDMMC 1-bit） |

### 引脚

```
e-Paper  MOSI=13  SCK=12  CS=11  DC=10  RST=9  BUSY=8  PWR=6
SD       CLK=39   CMD=41  D0=40
I2S      MCLK=14  BCLK=15 LRC=38 DOUT=45
ES8311   SDA=47   SCL=48
AUDIO_PWR=42  PA=46  VBAT_LATCH=17
BOOT_BTN=0  PWR_BTN=35
```

---

## 依赖库

| 库 | 版本 |
|----|------|
| [LVGL](https://github.com/lvgl/lvgl) | 9.2.2 |
| [ESP32-audioI2S](https://github.com/schreibfaul1/ESP32-audioI2S) | 3.4.5 |
| Arduino ESP32 core | ≥ 3.x |

配置 LVGL：将 `lv_conf_template.h` 复制为 `lv_conf.h` 并修改：

```c
#define LV_FONT_MONTSERRAT_12  1   // 用于播放/暂停图标
#define LV_COLOR_DEPTH         16
```

---

## SD 卡文件结构

```
/music/           ← 扫描此目录；若为空则扫描根目录
  ├── 许巍 - 旅行.mp3
  ├── 许巍 - 旅行.meta    ← 自动生成的 meta 缓存（含封面）
  └── ...
/.epd_frame       ← 上次 e-ink 帧（开机免刷新用）
```

MP3 文件名支持 `"艺术家 - 标题.mp3"` 格式自动解析，也支持 ID3v2 标签。

---

## 编译 & 刷机

### FQBN

```
esp32:esp32:esp32s3:FlashSize=8M,FlashMode=qio,PartitionScheme=custom,PSRAM=opi,CPUFreq=240,DebugLevel=none,USBMode=hwcdc,CDCOnBoot=cdc
```

### arduino-cli

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:FlashSize=8M,FlashMode=qio,PartitionScheme=custom,PSRAM=opi,CPUFreq=240,DebugLevel=none,USBMode=hwcdc,CDCOnBoot=cdc" \
  --output-dir /tmp/epaper_build \
  ESP32_EPAPER_PLAYER

esptool --chip esp32s3 --port /dev/cu.usbmodem* --baud 921600 write-flash \
  --flash-mode keep --flash-freq keep --flash-size keep \
  0x0000 /tmp/epaper_build/ESP32_EPAPER_PLAYER.ino.bootloader.bin \
  0x8000 /tmp/epaper_build/ESP32_EPAPER_PLAYER.ino.partitions.bin \
  0x10000 /tmp/epaper_build/ESP32_EPAPER_PLAYER.ino.bin
```

---

## 功能

- MP3 播放（ESP32-audioI2S），支持 ID3v2 标签、歌词（LRC 内嵌）
- e-ink 局部刷新 UI：专辑封面（Floyd-Steinberg 抖动）、标题、艺术家、进度条、歌词滚动
- 封面解码在 Core 0 独立任务，音频 `audio.loop()` 始终在 Core 1 不被阻塞
- **开机免刷屏**：保存上次帧到 SD，重启只做 partial update（仅更新变化像素）
- **暂停时后台预扫描**：Core 0 任务在暂停期间解析未缓存曲目的 ID3 text 标签，写入 `.meta` 文件
- NVS 断点续播：每 3 秒记录播放进度，重启后自动恢复

### 按键

| 按键 | 操作 | 功能 |
|------|------|------|
| BOOT | 短按 | 播放 / 暂停 |
| BOOT | 长按（>0.8s） | 上一首 |
| PWR  | 短按 | 下一首 |

---

## 分区表

`partitions.csv`（8 MB Flash）：

| 名称 | 类型 | 大小 |
|------|------|------|
| nvs | data/nvs | 20 KB |
| otadata | data/ota | 8 KB |
| app0 | app/ota_0 | **6.9 MB** |
| coredump | data/coredump | 64 KB |
