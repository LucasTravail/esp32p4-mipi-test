# --- LVGL Pro Editor project -----------------------------------------------------------
# Create/open the editor project in ../ui (project name: esp32p4_ui) and export the C code into
# that same folder: every .c file below ../ui is compiled into this component, and every folder
# that contains a header is added to the include path (run `idf.py reconfigure` after the first export). Without an export the built-in demo UI is used.
set(UI_DIR ${CMAKE_CURRENT_LIST_DIR}/../ui)
file(GLOB_RECURSE UI_SRCS ${UI_DIR}/*.c)
file(GLOB_RECURSE UI_HDRS ${UI_DIR}/*.h)
# the editor project also contains a PC simulator and build folders: not part of the firmware
list(FILTER UI_SRCS EXCLUDE REGEX "/(sim|build|preview-build)/")
list(FILTER UI_HDRS EXCLUDE REGEX "/(sim|build|preview-build)/")
set(UI_INCLUDE_DIRS "")
foreach(h ${UI_HDRS})
    get_filename_component(d ${h} DIRECTORY)
    list(APPEND UI_INCLUDE_DIRS ${d})
endforeach()
list(REMOVE_DUPLICATES UI_INCLUDE_DIRS)

idf_component_register(SRCS "main.c" ${UI_SRCS}
                       INCLUDE_DIRS "." ${UI_INCLUDE_DIRS}
                       REQUIRES esp_lcd esp_mm esp_driver_gpio esp_driver_i2c soc esp_system lvgl__lvgl)

# the generated code includes "lvgl.h"
if(COMPONENT_LIB)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)
endif()
