# LVGL Pro Editor project

1. Open the Editor (desktop app or VS Code) and create a project **in this folder** (`ui/`), project name `esp32p4_ui`.
2. Design your screens; one screen must be named `main_screen` (file `main_screen.xml`).
3. Export the C code into this folder (hammer / "Compile and export code", or the CLI).
4. `idf.py build`: the firmware picks the generated code up automatically
   (`esp32p4_ui.h`, `esp32p4_ui_init("")`, `main_screen_create()`); without it the built-in demo UI is shown.

Display: 466x466 round, RGB888. Touch: pointer device, already wired to LVGL.
