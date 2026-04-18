idf_component_register(
    SRCS
        "main.c"
        "esp_lcd_sh8601.c"
        "lv_font_montserrat_120.c"
        "lv_font_montserrat_150.c"
        "lv_font_montserrat_40_limited.c"
    INCLUDE_DIRS
        "."
    EMBED_FILES
        "moco_jib_logo_600x310_dark.bin"
        "moco_jib_logo_600x310_light.bin"
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)
