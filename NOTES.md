# Project notes (state at 2026-10-31, evening) — read this first in a new session

ESP32-P4 (rev < v3.0), ESP-IDF 5.5.5, LVGL 9.5. Round 466x466 AMOLED (CH13613 / TT151AMC82C, 1 DSI lane @ 250 Mbps,
do not raise it), CST9217 touch. Digital-caliper UI: round gauge, +/-150 mm = one full turn. A second chip, an ESP8684
(ESP32-C2 family, "the C2"), gives Wi-Fi/BLE and is flashed *by the P4* (no USB of its own).

## Build / flash (Windows)
Env: IDF_PATH=C:\esp\v5.5.5\esp-idf, IDF_TOOLS_PATH=C:\Espressif, IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.5.5\venv, then
`cmd /d /c "call C:\esp\v5.5.5\esp-idf\export.bat && idf.py -DCCACHE_ENABLE=0 build -- -j2 && python scripts\flash_via_1200.py --project-dir . --app-port COM39 --rom-port COM38"`.
COM ports change per machine: check Device Manager (app CDC port + ROM port). Delete `sdkconfig` after editing `sdkconfig.defaults`.
The 1200-bps CDC trick reboots into the ROM loader: never break it. Logs: USB CDC (app port), `cdc_log()`.
- On the 2nd PC, `export.bat` first failed (missing espidf.constraints.v5.5.txt): fixed by running `install.bat esp32p4` once.
- The Windows toolchain randomly dies with "internal compiler error ... try_forward_edges": use `-DCCACHE_ENABLE=0 ... -- -j2` and just
  rerun, the build resumes.
- If a COM port "cannot be configured / device not functioning" after a reflash: unplug/replug the USB cable.

## How the display is driven (main/main.c) — hard-won, do not redo
- IDF DPI video path stalls after 1 frame: not used. Pixels go by DCS RAMW 0x2C / RAMWC 0x3C in HS (`dcs_lw_tx = 0`).
- Panel needs SWIRE 0x6C=0xA5 and brightness 0x51=0xFF, else black. Windows must be even-aligned, COL_OFFSET 6.
- The IDF DBI driver enables command ACK: a bus turn-around after EVERY packet was the real speed limit (~7 MB/s).
  `cmd_mode_cfg.ack_rqst_en = 0` + own writer `dsi_pixels()` (payload first, header last; whole payload must fit the host FIFO)
  + RGB565 (COLMOD 0x3A=0x05): a full frame now takes ~24 ms.
- Swipe navigation = compositor in PSRAM (`nav_*`): the current screen is mirrored into `s_shadow` by the flush callback, menus
  are rendered once into PSRAM images, and each animation frame is streamed straight to the panel (parallax, spring on release,
  TE-synchronised). LVGL is not running during a drag. 32 MB PSRAM, ~30 MB free.
- LVGL memory pool is 64 KB (CONFIG_LV_MEM_SIZE_KILOBYTES): a big UI needs ~160 KB or it hangs at boot.
- Value font: main/value_font.ttf = Orbitron (static instance, weight 600), rendered with tiny_ttf at 68 px.

## Display glitches: it was hardware (important lesson)
Heavy screen glitches / "DSI timeout x438" + frozen log + dead touch appeared after rework. The ORIGINAL committed firmware showed the same
on this board, so not a software bug. Causes found: bad USB-C ground (fixed with a wire/soldering), weak 3V3 supply noise.
It was much better after adding ~50 uF on the 3V3 output (U9) + 10 uF on VBUS; it is still worse on USB 5 V than on battery.
Suspects left: buck-boost U9 MODE pin tied low through R19 (power-save mode, try forced PWM), output cap only 10 uF originally,
very poor ground plane, DSI traces with 2 vias each. Next PCB: 4 layers, solid ground plane, 100 ohm DSI pair matched inside the pair
(inter-lane matching is loose at 250 Mbps), no/symmetric vias, bigger and more local decoupling.

## Power circuit (read from the schematic) — KNOWN FLAWS, fix on the next PCB
- Latch: SW3 (battery side) -> Q9 pulls the gate of PMOS Q12 low; Q10 driven by PWR_OFF (GPIO17) HIGH also pulls it low = HOLD ON.
  GPIO17 low = power off (battery only). PWR_ON (GPIO18) = divider 100k/330k of the SW3 node, HIGH while the button is pressed.
- The USB-C VBUS pins and the drain of Q12 are the SAME net (VUSB), which also feeds the charger input and the buck-boost (U9).
  So on USB, 5 V flows back into +BATT through Q12's body diode (~4.4 V) or through Q12 itself when held on (~4.9 V; the P4 then
  reads 4.86 V on BAT_ADC). **A LiPo must not be left connected while on USB** (overcharge, charger U7 bypassed). Needs a real
  power path (USB diode/ideal diode + back-to-back PMOS).
- GPIO18 can see ~3.7 V (above the 3.6 V limit) if the button is pressed while +BATT is ~4.9 V (USB): add a clamp / change the divider.
- Charger U7: STAT (net CHRG) only drives LED D1, not wired to the P4 (and it sits at +BATT level: needs a level shifter).
  No way today to know "charging" nor a correct % while charging. Next PCB: add a fuel gauge (MAX17048 or BQ27441, I2C) and wire STAT.
- U9 (buck-boost, 0.47 uH, FB 511k/91k ~ 3.31 V): measured 3.1 V on the bench (overload suspected, measure the board current).

## Hardware facts / pins
LCD: RESET=14, VCI_EN=15, TE=13. Touch: SDA=7 SCL=5 RST=3 INT=12 (poll every 30 ms). P4 LED (heartbeat, 2 Hz) = GPIO21.
Buzzer: PASSIVE, GPIO43 -> 1k -> 2N7002 gate, buzzer between 3V3 and drain: notes are LEDC PWM (square wave), a steady level is silent.
Main switch GPIO0 (internal pull-up, pressed = 0). Power-on switch GPIO18 (pressed = 1, board has its own pull-down, no internal pull).
PWR_OFF GPIO17 (latch, see above). Battery 1S LiPo: BAT_ADC=GPIO23 via 2x200k divider (`BAT_CAL` to calibrate with a multimeter on +BATT).
AS5311 (magnetic linear encoder, quadrature): A=GPIO33, B=GPIO37, Index=GPIO38 (unused). UART0 console is DISABLED
(`CONFIG_ESP_CONSOLE_NONE`) because its default pins GPIO37/38 are the encoder B/Index lines.
ESP8684: UART RX=19 TX=20 (P4 side UART1), EN=32, BOOT=28 (GPIO9). GPIO8 has a 10k pull-up to 3V3 (needed for download mode).
Its LED_ACT is GPIO18 (on the C2). 4 MB flash, MAC C0:5D:89:3F:70:8C, 40 MHz crystal.

## Firmware behaviour (main/main.c, main/c2.c, main/encoder.c)
- Boot: heartbeat task, `power_boot()` takes the latch at once (GPIO17 high) and plays the rising "power on" tune, `encoder_init()`,
  USB CDC, C2 task (`c2_run`: BOOT high + EN pulse, starts `c2_link_task` and `c2_inputs_task`), then LCD/LVGL.
- Power switch (GPIO18): held 3 s (after the press that switched the board on has been released) = falling "power off" tune and
  GPIO17 low (on USB the board stays alive, GPIO17 goes back high after 2 s). Short press = mm <-> inch (`g_unit_inch`, label "mm"/"in",
  inches with 3 decimals), short beep.
- Main switch (GPIO0): press = zero the measurement (`encoder_zero()`) + two short beeps.
- AS5311 decoded by PCNT (x4, glitch filter 100 ns, accum_count). `ENC_COUNTS_PER_MM` 2048 (2 mm pole pair, 4096 edges), `ENC_DIR` = +/-1.
  NOT yet tested with a real magnetic ruler: check direction and scale. `CALIPER_DEMO` 0 = real reading, 1 = fake sweep. `ENC_ENABLE`.
- Battery icon is real now (`BAT_SIM 0`): % from a LiPo voltage curve, 5 steps of 20 %. Only valid on battery (USB reads ~4.9 V).
- Web button "Eteindre" (C2 page) = same as the 3 s power-off. Web button "Buzzer" and BLE connect play Ode to Joy; BLE disconnect = falling arpeggio.
- Compile-time switches: `C2_PROBE` (main.c; 1 = flash c2_blink.bin to the C2 at boot, then set back to 0), `C2_ENABLE`
  (c2.c; 0 holds the C2 in reset, no radio), `POWER_PINS` (c2.c; 0 = never touch GPIO17/18), `ENC_ENABLE`, `CALIPER_DEMO`, `BAT_SIM`.
- `reset reason: N` is logged at boot (1 power-on, 4 panic, 5/6/7 watchdog, 9 brownout).

## ESP8684 firmware (c2_blink/, ESP-IDF project, target esp32c2)
Open Wi-Fi AP "Caliper-C2" (channel 6, no password) + web page http://192.168.4.1/ (Buzzer, Eteindre) + BLE "Caliper-C2" (NimBLE, connectable).
UART0 to the P4 (C2 TX GPIO20 -> P4 GPIO19, C2 RX GPIO19 <- P4 GPIO20), console and logs off. Lines C2 -> P4: `$BUZZ`, `$OFF`, `$CONN`,
`$DISC`, `$BLE init=.. sync=.. addr=.. adv=..` (status every 3 s; adv=0 means advertising started). P4 -> C2: `$IN ...` is no longer sent.
Its LED (GPIO18): 2 Hz blink = AP up, steady with a client, off briefly on a web press.
BLE advertising reported OK by the C2, but "Caliper-C2" was NOT seen from the phone/PC scanner when tested: unresolved.
**Reflash the C2**: `powershell c2_blink\build.ps1` (merged image: bootloader+partition table+app, flashed at 0x0; do NOT flash
build\c2_blink.bin alone, it is the app only) copies main/c2_blink.bin; then set `C2_PROBE 1`, build/flash the P4, wait for the log line
"c2: flashed and verified", set `C2_PROBE 0`, build/flash the P4 again. LVGL is paused while flashing (`g_lvgl_pause`), otherwise the P4 panicked.
After editing blink.c keep escapes like "\n" in C strings intact (a past tool mangled them into real newlines).

## UI state
Caliper gauge (dark -> light gradient by position, round caps, filled over +/-150 mm), real battery icon, 4-way swipe to
placeholder menus. Neon Pulse (ChatGPT-generated theme) was tried and abandoned. `UI_CALIPER` / `UI_MIRE`, `USE_LVGL` in main.c.

## Next ideas
Test the AS5311 with a real magnetic ruler (direction, scale, noise); calibrate `BAT_CAL`; fix the power path and the charger indication
(see above, new PCB); BLE visibility; decide what the web page should show (position, battery); real menus; 60 fps (frame write ~24 ms).
