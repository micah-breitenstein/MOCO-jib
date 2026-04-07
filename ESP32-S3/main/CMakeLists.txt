idf_component_register(
    SRCS
        "main.c"
        "esp_lcd_sh8601.c"
        "lv_font_montserrat_120.c"
    INCLUDE_DIRS
        "."
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)
