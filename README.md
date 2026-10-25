# ESP32-P4 MIPI test: red / green / blue full-screen (ESP-IDF 5.5.5)

CH13613 / TT151AMC82C 466x466 AMOLED, 1 lane, 250 Mbps. The firmware shows red, green, blue, white, black
in a loop (1.5 s each) by writing the panel's own GRAM in **DSI command mode**: DCS RAMW (0x2C) then
RAMWC (0x3C), 40 px per packet, **sent in HS** (`cmd_mode_cfg.dcs_lw_tx = 0`). A full screen takes ~90 ms.
USB: TinyUSB CDC ACM; a **1200 baud** line-coding request reboots into the ROM loader.
Logs go to the USB CDC port (and UART0 GPIO37/38, 115200). MIPI PHY (2.5 V) and panel VBAT are external.

Things learnt on this hardware (do not rediscover them):
- The IDF DPI video path (`esp_lcd_new_panel_dpi`) streams exactly ONE frame and then stalls (bridge FIFO stays
  full, host reports `dpi_buff_pld_under`): not used. The host pattern generator (bars) does work.
- The panel stays black until the power IC for ELVDD/ELVSS is enabled: write `0x6C = 0xA5` (SWIRE manual).
- RAMW/RAMWC must be sent in HS (datasheet): in LP the writes are silently ignored.
- The IDF DCS long-write helper fills the payload FIFO before the header: keep packets small (<= ~120 bytes).
- ALLPOFF (0x22) is NOT left by NORON (0x13): do not use it for tests, hardware-reset the panel instead.

## Build
ESP-IDF terminal (or VS Code: ESP-IDF: Build):
    idf.py set-target esp32p4   # once
    idf.py build
`sdkconfig.defaults` selects P4 rev < v3.0 (v1.3), 8 MB DIO 40 MHz flash, PSRAM, TinyUSB CDC.
Delete `sdkconfig` after editing the defaults.

## First flash (manual)
Hold BOOT (GPIO35, pin 66) to GND, briefly pull RESET (CHIP_PU, pin 103) to GND, release RESET, then BOOT.
(Pin 68 is GPIO36, not RESET.) Then `idf.py -p COMxx flash` on the ROM-loader port.

## Next flashes (no BOOT/RESET)
VS Code: Terminal > Run Task > **P4: CDC 1200 -> ROM port -> flash -> back to firmware**.
It prompts for the current firmware CDC port (default COM37) and the ROM-loader port (default COM36);
check both in Device Manager (the ROM port is only visible while in ROM mode: run once with --no-flash to see it).
CLI equivalent:
    python scripts/flash_via_1200.py --project-dir . --app-port COM37 --rom-port COM36
No VID/PID is assumed. Needs IDF_PATH and pyserial (ESP-IDF environment).
The standard "Flash" button does NOT send the 1200 baud request.
