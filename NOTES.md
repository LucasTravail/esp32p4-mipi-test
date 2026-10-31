# Project notes (state at 2026-10-31) — read this first in a new session

ESP32-P4 (rev < v3.0), ESP-IDF 5.5.5, LVGL 9.5. Round 466x466 AMOLED (CH13613 / TT151AMC82C, 1 DSI lane @ 250 Mbps,
do not raise it), CST9217 touch. Digital-caliper UI: round gauge, +/-150 mm = one full turn.

## Build / flash (Windows)
Env: IDF_PATH=C:\esp\v5.5.5\esp-idf, IDF_TOOLS_PATH=C:\Espressif, IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.5.5\venv, then
`cmd /d /c "call C:\esp\v5.5.5\esp-idf\export.bat && idf.py build && python scripts\flash_via_1200.py --project-dir . --app-port COM17 --rom-port COM16"`.
COM ports change per machine: check Device Manager (app CDC port + ROM port). Delete `sdkconfig` after editing `sdkconfig.defaults`.
The 1200-bps CDC trick reboots into the ROM loader: never break it. Logs: USB CDC (COM17).

## How the display is driven (main/main.c) — hard-won, do not redo
- IDF DPI video path stalls after 1 frame: not used. Pixels go by DCS RAMW 0x2C / RAMWC 0x3C in HS (`dcs_lw_tx = 0`).
- Panel needs SWIRE 0x6C=0xA5 and brightness 0x51=0xFF, else black. Windows must be even-aligned, COL_OFFSET 6.
- The IDF DBI driver enables command ACK: a bus turn-around after EVERY packet was the real speed limit (~7 MB/s).
  `cmd_mode_cfg.ack_rqst_en = 0` + own writer `dsi_pixels()` (payload first, header last; whole payload must fit the host FIFO)
  + RGB565 (COLMOD 0x3A=0x05): a full frame now takes ~24 ms.
- Swipe navigation = compositor in PSRAM (`nav_*`): the current screen is mirrored into `s_shadow` by the flush callback, menus
  are rendered once into PSRAM images, and each animation frame is streamed straight to the panel (parallax, spring on release,
  TE-synchronised). LVGL is not running during a drag. 32 MB PSRAM, ~30 MB free.
- LVGL memory pool is 64 KB (CONFIG_LV_MEM_SIZE_KILOBYTES): a big UI (e.g. the abandoned Neon Pulse) needs ~160 KB or it hangs at boot.

## Hardware facts
LCD: RESET=14, VCI_EN=15, TE=13. Touch: SDA=7 SCL=5 RST=3 INT=12 (poll every 30 ms). LED=21. Buzzer BUZZ_IO=GPIO43.
Battery 1S LiPo: BAT_ADC=GPIO23 via 2x200k divider (reads ~4.45 V on a full cell: ADC error, needs a multimeter calibration).
AS5311 position sensor: A=GPIO33, B=GPIO37, Ind=GPIO38 (not read yet). MainSwitch=GPIO0.
ESP8684 (Wi-Fi co-processor): UART RX=19 TX=20, EN=32, BOOT=28. Link works (ROM boot log received) but its GPIO8 floats low,
so it cannot enter download mode: needs a ~10k pull-up from GPIO8 (pin 14) to 3V3 (hardware, planned for the weekend).
Its flash is blank. Test code: main/c2.c (`C2_PROBE`, currently off). Then flash a blinky (LED on its GPIO18), build it with the 40 MHz crystal.

## UI state
Caliper gauge (dark -> light gradient by position, round caps, battery icon demo cycling charge/100..20 %), 4-way swipe to
placeholder menus. Demo values sweep -150..+150 mm (24 s). Battery icon runs a demo (`BAT_SIM 1`); no real sensor reading wired.
Neon Pulse (ChatGPT-generated theme) was tried and abandoned. `UI_CALIPER` / `UI_MIRE` switches, `USE_LVGL` in main.c.

## Next ideas
Read the AS5311 and publish real mm values; calibrate battery; zero/hold buttons; real menus; ESP8684 programming (needs the pull-up);
60 fps (frame write is ~24 ms, panel scans at 16.7 ms).
