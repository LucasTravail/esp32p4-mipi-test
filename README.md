# ESP32-P4 MIPI hardware test pattern (ESP-IDF 5.5.5)

Vertical colour bars from the P4 MIPI-DSI **hardware pattern generator** on a CH13613 / TT151AMC82C 466x466 panel
(1 lane, 250 Mbps, DPI 6 MHz). USB: TinyUSB CDC ACM; a **1200 baud** line-coding request reboots into the ROM loader.
Logs on UART0 (GPIO37/38, 115200). MIPI PHY (2.5 V) and panel VBAT are external: no internal LDO is configured.
Config is not fully validated: rare residual glitches were seen under Arduino.

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
