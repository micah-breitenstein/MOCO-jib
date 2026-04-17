#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_lcd_sh8601.h"
#include "lvgl.h"
#include "driver/i2c.h"
#include "nvs.h"
#include "nvs_flash.h"

extern const lv_font_t lv_font_montserrat_150;
extern const lv_font_t lv_font_montserrat_120;
extern const lv_font_t lv_font_montserrat_48;

static const char *TAG = "RIG";

#define LCD_HOST SPI2_HOST
#define LCD_BIT_PER_PIXEL 16

#define PIN_NUM_LCD_CS (GPIO_NUM_9)
#define PIN_NUM_LCD_PCLK (GPIO_NUM_10)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_11)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_12)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_13)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_14)
#define PIN_NUM_LCD_RST (GPIO_NUM_21)

#define LCD_H_RES 600
#define LCD_V_RES 450
#define LCD_QSPI_WRITE_CMD 0x02U

#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 10
#define LVGL_TASK_STACK_SIZE (6 * 1024)
#define LVGL_TASK_PRIORITY 2
#define LVGL_BUF_HEIGHT (LCD_V_RES / 10)
#define TEST_FORCE_CONTROLLER_ERROR 0
#define TEST_FORCE_CONTROLLER_ERROR_DELAY_MS 3000

#define STATUS_UART_PORT UART_NUM_1
#define STATUS_UART_RX_PIN GPIO_NUM_40
#define STATUS_UART_TX_PIN GPIO_NUM_41
#define STATUS_UART_BAUDRATE 115200
#define STATUS_UART_BUF_SIZE 2048
#define STATUS_SIGNAL_TIMEOUT_MS 3500
#define MODE_MESSAGE_DURATION_MS 1800
#define TIMELAPSE_MESSAGE_DURATION_MS 6000
#define DRONE_STICK_MIN_VISIBLE_PULSE_MS 20

/* ---------- Touch / I2C ---------- */
#define TOUCH_I2C_PORT    I2C_NUM_0
#define TOUCH_I2C_SDA     GPIO_NUM_47
#define TOUCH_I2C_SCL     GPIO_NUM_48
#define TOUCH_I2C_FREQ_HZ 400000
#define TOUCH_FT6336_ADDR 0x38
#define LONG_PRESS_MS     500

/* ---------- NVS ---------- */
#define NVS_NAMESPACE "rig_cfg"

/* ---------- Settings data model ---------- */

typedef enum {
    SGRP_TIMELAPSE = 0,
    SGRP_SYSTEM,
    SGRP_COUNT,
} SettingGroup;

static const char *setting_group_names[SGRP_COUNT] = {
    "TIMELAPSE",
    "SYSTEM",
};

typedef enum {
    SETTING_TL_INTERVAL = 0,
    SETTING_TL_STEPDIST,
    SETTING_RUMBLE_MUTE,
    SETTING_BRIGHTNESS,
    SETTING_LOGO_THEME,
    SETTING_COUNT,
} SettingId;

typedef enum {
    STYPE_INT_RANGE = 0,
    STYPE_BOOL,
} SettingType;

typedef struct {
    const char *name;
    const char *unit;
    SettingGroup group;
    SettingType  type;
    int value;
    int default_val;
    int min_val;
    int max_val;
    int step;
    bool mega_backed;
    bool triggers_rumble;
} SettingDef;

static SettingDef settings[SETTING_COUNT] = {
    [SETTING_TL_INTERVAL] = { "Interval",     "s",  SGRP_TIMELAPSE, STYPE_INT_RANGE, 15,  15,  1,   99,  1,  true,  true  },
    [SETTING_TL_STEPDIST] = { "Step Dist",    "ms", SGRP_TIMELAPSE, STYPE_INT_RANGE, 100, 100, 20,  150, 10, true,  true  },
    [SETTING_RUMBLE_MUTE] = { "Rumble Mute",  "",   SGRP_SYSTEM,    STYPE_BOOL,      0,   0,   0,   1,   1,  true,  false },
    [SETTING_BRIGHTNESS]  = { "Brightness",   "%",  SGRP_SYSTEM,    STYPE_INT_RANGE, 80,  80,  10,  100, 5,  false, false },
    [SETTING_LOGO_THEME]  = { "Logo Theme",   "",   SGRP_SYSTEM,    STYPE_BOOL,      0,   0,   0,   1,   1,  false, false },
};

/* NVS keys for the 5 settings (short for 15-char NVS limit) */
static const char *nvs_keys[SETTING_COUNT] = { "tl_int", "tl_step", "r_mute", "bright", "theme" };

/* ---------- Settings menu state ---------- */
static bool                settings_visible = false;
static bool                editor_visible = false;
static SettingId           editor_setting_id = SETTING_TL_INTERVAL;
static esp_lcd_panel_io_handle_t panel_io_global = NULL;

/* Touch state */
static bool    touch_pressed = false;
static int64_t touch_press_start_us = 0;
static bool    long_press_fired = false;

typedef enum {
    DISPLAY_MODE_MANUAL = 0,
    DISPLAY_MODE_DRONE,
    DISPLAY_MODE_TIMELAPSE,
    DISPLAY_MODE_BOUNCE,
    DISPLAY_MODE_ERROR,
} DisplayMode;

typedef enum {
    DRONE_LIFT_NEUTRAL = 0,
    DRONE_LIFT_UP,
    DRONE_LIFT_DOWN,
} DroneLiftDisplayState;

typedef enum {
    DRONE_HORIZ_NEUTRAL = 0,
    DRONE_HORIZ_LEFT,
    DRONE_HORIZ_RIGHT,
} DroneHorizontalDisplayState;

typedef enum {
    DRONE_TILT_NEUTRAL = 0,
    DRONE_TILT_UP,
    DRONE_TILT_DOWN,
} DroneTiltDisplayState;

static SemaphoreHandle_t lvgl_mux = NULL;
static lv_obj_t *label_moco = NULL;
static lv_obj_t *label_jib = NULL;
static lv_obj_t *status_label = NULL;
static lv_obj_t *drone_title_label = NULL;
static lv_obj_t *drone_subtitle_label = NULL;
static lv_obj_t *drone_left_ring = NULL;
static lv_obj_t *drone_right_ring = NULL;
static lv_obj_t *drone_left_stick = NULL;
static lv_obj_t *drone_right_stick = NULL;
static lv_obj_t *drone_center_line = NULL;
static lv_obj_t *drone_precision_label = NULL;
static lv_obj_t *drone_boost_label = NULL;
static lv_obj_t *drone_precision_state_box = NULL;
static lv_obj_t *drone_boost_state_box = NULL;
static lv_obj_t *drone_flowlapse_bar = NULL;
static lv_obj_t *drone_flowlapse_fill = NULL;
static lv_obj_t *drone_flowlapse_label = NULL;
static lv_obj_t *drone_flowlapse_waypoint_label = NULL;
static lv_obj_t *settings_list_panel = NULL;
static lv_obj_t *settings_editor_panel = NULL;
static lv_obj_t *editor_title_label = NULL;
static lv_obj_t *editor_value_label = NULL;
static bool status_error_active = false;
static bool mode_message_active = false;
static uint64_t mode_message_until_ms = 0;
static DisplayMode current_display_mode = DISPLAY_MODE_MANUAL;
static DisplayMode last_non_error_mode = DISPLAY_MODE_MANUAL;
static bool controller_connected = false;
static bool emergency_stop_active = false;
static bool restore_mode_after_message = false;
static DisplayMode mode_to_restore_after_message = DISPLAY_MODE_MANUAL;
static DroneLiftDisplayState drone_lift_display_state = DRONE_LIFT_NEUTRAL;
static DroneHorizontalDisplayState drone_swing_display_state = DRONE_HORIZ_NEUTRAL;
static DroneHorizontalDisplayState drone_pan_display_state = DRONE_HORIZ_NEUTRAL;
static DroneTiltDisplayState drone_tilt_display_state = DRONE_TILT_NEUTRAL;
static int drone_swing_display_percent = 0;
static int drone_lift_display_percent = 0;
static int drone_pan_display_percent = 0;
static int drone_tilt_display_percent = 0;
static bool drone_left_stick_pressed = false;
static bool drone_right_stick_pressed = false;
static uint64_t drone_swing_pulse_start_ms = 0;
static uint64_t drone_lift_pulse_start_ms = 0;
static uint64_t drone_pan_pulse_start_ms = 0;
static uint64_t drone_tilt_pulse_start_ms = 0;
static int last_logged_swing_dir = 0;
static int last_logged_lift_dir = 0;
static int last_logged_pan_dir = 0;
static int last_logged_tilt_dir = 0;
static bool drone_precision_modifier_active = false;
static bool drone_boost_modifier_active = false;
static bool flowlapse_active = false;
static int flowlapse_progress_percent = 0;
static char flowlapse_status_text[48] = "FLOWLAPSE READY";
static int flowlapse_waypoint_current = 0;
static int flowlapse_waypoint_total = 0;
static bool flowlapse_waypoint_indicator_active = false;
static bool flowlapse_playback_active = false;

#define DRONE_LEFT_STICK_BASE_X 145
#define DRONE_LEFT_STICK_BASE_Y 265
#define DRONE_RIGHT_STICK_BASE_X 435
#define DRONE_RIGHT_STICK_BASE_Y 265
#define DRONE_LIFT_INDICATOR_OFFSET 46
#define DRONE_HORIZ_INDICATOR_OFFSET 46

#define FLOWLAPSE_BAR_NORMAL_X 60
#define FLOWLAPSE_BAR_NORMAL_Y 144
#define FLOWLAPSE_BAR_NORMAL_W (LCD_H_RES - 120)
#define FLOWLAPSE_BAR_NORMAL_H 36

#define FLOWLAPSE_LABEL_NORMAL_Y 150

#define FLOWLAPSE_BAR_PLAYBACK_X 20
#define FLOWLAPSE_BAR_PLAYBACK_Y 184
#define FLOWLAPSE_BAR_PLAYBACK_W (LCD_H_RES - 40)
#define FLOWLAPSE_BAR_PLAYBACK_H (FLOWLAPSE_BAR_NORMAL_H * 4)
#define FLOWLAPSE_LABEL_PLAYBACK_Y (FLOWLAPSE_BAR_PLAYBACK_Y + ((FLOWLAPSE_BAR_PLAYBACK_H - 36) / 2))

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x26, (uint8_t[]){0x0A}, 1, 0},
    {0x24, (uint8_t[]){0x80}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0xC2, (uint8_t[]){0x00}, 1, 10},
    {0x35, (uint8_t[]){0x00}, 0, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x11, (uint8_t[]){0x00}, 0, 80},
    {0x2A, (uint8_t[]){0x00, 0x10, 0x01, 0xD1}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x02, 0x57}, 4, 0},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x36, (uint8_t[]){0x30}, 1, 10},
    {0x51, (uint8_t[]){0x00}, 1, 0},
};

static bool lvgl_flush_ready_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_drv);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    const int offset_x1 = area->x1;
    const int offset_x2 = area->x2;
    const int offset_y1 = area->y1 + 16;
    const int offset_y2 = area->y2 + 16;
    esp_lcd_panel_draw_bitmap(panel_handle, offset_x1, offset_y1, offset_x2 + 1, offset_y2 + 1, color_map);
}

static void lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    (void)disp_drv;
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static esp_err_t sh8601_tx_param_qspi(esp_lcd_panel_io_handle_t io_handle, uint8_t cmd, const void *param, size_t param_size)
{
    uint32_t lcd_cmd = ((uint32_t)LCD_QSPI_WRITE_CMD << 24) | ((uint32_t)cmd << 8);
    return esp_lcd_panel_io_tx_param(io_handle, lcd_cmd, param, param_size);
}

static void anim_set_panel_brightness(void *obj, int32_t value)
{
    if (value < 0) {
        value = 0;
    }
    if (value > 255) {
        value = 255;
    }
    uint8_t level = (uint8_t)value;
    esp_lcd_panel_io_handle_t io_handle = (esp_lcd_panel_io_handle_t)obj;
    (void)sh8601_tx_param_qspi(io_handle, 0x51, &level, 1);
}

static void lvgl_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "LVGL task started");
    while (1) {
        if (xSemaphoreTake(lvgl_mux, portMAX_DELAY) == pdTRUE) {
            uint32_t delay_ms = lv_timer_handler();
            xSemaphoreGive(lvgl_mux);
            if (delay_ms > LVGL_TASK_MAX_DELAY_MS) {
                delay_ms = LVGL_TASK_MAX_DELAY_MS;
            } else if (delay_ms < 1) {
                delay_ms = 1;
            }
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

/* ================================================================
 *  NVS helpers
 * ================================================================ */

static void load_settings_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    for (int i = 0; i < SETTING_COUNT; i++) {
        int32_t v;
        if (nvs_get_i32(h, nvs_keys[i], &v) == ESP_OK) {
            settings[i].value = (int)v;
        }
    }
    nvs_close(h);
}

static void save_setting_to_nvs(SettingId id)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, nvs_keys[id], (int32_t)settings[id].value);
    nvs_commit(h);
    nvs_close(h);
}

static void reset_all_settings_to_defaults(void)
{
    for (int i = 0; i < SETTING_COUNT; i++) {
        settings[i].value = settings[i].default_val;
        save_setting_to_nvs((SettingId)i);
    }
}

/* ================================================================
 *  Brightness helper
 * ================================================================ */

static void apply_brightness(void)
{
    if (!panel_io_global) return;
    int pct = settings[SETTING_BRIGHTNESS].value;
    if (pct < 10) pct = 10;
    if (pct > 100) pct = 100;
    uint8_t level = (uint8_t)((pct * 255) / 100);
    uint32_t lcd_cmd = ((uint32_t)LCD_QSPI_WRITE_CMD << 24) | ((uint32_t)0x51 << 8);
    esp_lcd_panel_io_tx_param(panel_io_global, lcd_cmd, &level, 1);
}

/* ================================================================
 *  Theme application
 * ================================================================ */

static void apply_theme(void)
{
    bool light = (settings[SETTING_LOGO_THEME].value != 0);
    lv_color_t bg  = light ? lv_color_white() : lv_color_black();
    lv_color_t fg  = light ? lv_color_black() : lv_color_white();

    lv_obj_set_style_bg_color(lv_scr_act(), bg, LV_PART_MAIN);
    if (label_moco) lv_obj_set_style_text_color(label_moco, fg, LV_PART_MAIN);
    if (label_jib)  lv_obj_set_style_text_color(label_jib,  fg, LV_PART_MAIN);
}

/* ================================================================
 *  Touch driver (FT6336 over I2C)
 * ================================================================ */

static bool read_ft6336_touch(uint16_t *x, uint16_t *y)
{
    uint8_t data[5] = {0};
    uint8_t reg = 0x02;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_FT6336_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_FT6336_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return false;

    uint8_t tp = data[0] & 0x0F;
    if (tp == 0) return false;

    uint16_t raw_x = ((data[1] & 0x0F) << 8) | data[2];
    uint16_t raw_y = ((data[3] & 0x0F) << 8) | data[4];
    *x = raw_y;
    *y = 450 - raw_x;
    return true;
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    uint16_t tx, ty;
    if (read_ft6336_touch(&tx, &ty)) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = tx;
        data->point.y = ty;
        if (!touch_pressed) {
            touch_pressed = true;
            touch_press_start_us = esp_timer_get_time();
            long_press_fired = false;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        touch_pressed = false;
        long_press_fired = false;
    }
}

/* ================================================================
 *  UART TX — send SET commands to MEGA
 * ================================================================ */

static void send_set_command(SettingId id)
{
    char cmd[48];
    switch (id) {
    case SETTING_TL_INTERVAL:
        snprintf(cmd, sizeof(cmd), "SET:TL_INT:%d\n", settings[id].value);
        break;
    case SETTING_TL_STEPDIST:
        snprintf(cmd, sizeof(cmd), "SET:TL_STEP:%d\n", settings[id].value);
        break;
    case SETTING_RUMBLE_MUTE:
        snprintf(cmd, sizeof(cmd), "SET:RUMBLE_MUTE:%d\n", settings[id].value);
        break;
    default:
        return;
    }
    uart_write_bytes(STATUS_UART_PORT, cmd, strlen(cmd));
}

/* ================================================================
 *  Settings UI — forward declarations
 * ================================================================ */

static void close_settings_menu(void);
static void open_editor(SettingId id);
static void close_editor(void);
static void set_drone_mode_visible(bool visible);

/* ================================================================
 *  Settings UI — Value editor (full screen)
 * ================================================================ */

static void format_setting_value(SettingId id, char *buf, size_t sz)
{
    const SettingDef *s = &settings[id];
    if (s->type == STYPE_BOOL) {
        if (id == SETTING_RUMBLE_MUTE) {
            snprintf(buf, sz, "%s", s->value ? "ON" : "OFF");
        } else if (id == SETTING_LOGO_THEME) {
            snprintf(buf, sz, "%s", s->value ? "LIGHT" : "DARK");
        } else {
            snprintf(buf, sz, "%s", s->value ? "ON" : "OFF");
        }
    } else {
        snprintf(buf, sz, "%d%s", s->value, s->unit);
    }
}

static void update_editor_value_label(void)
{
    if (!editor_value_label) return;
    char buf[16];
    format_setting_value(editor_setting_id, buf, sizeof(buf));
    lv_label_set_text(editor_value_label, buf);
}

static void editor_dec_cb(lv_event_t *e)
{
    (void)e;
    SettingDef *s = &settings[editor_setting_id];
    s->value -= s->step;
    if (s->value < s->min_val) s->value = s->min_val;
    update_editor_value_label();
    if (!s->mega_backed) {
        save_setting_to_nvs(editor_setting_id);
        if (editor_setting_id == SETTING_BRIGHTNESS) apply_brightness();
        if (editor_setting_id == SETTING_LOGO_THEME) apply_theme();
    }
}

static void editor_inc_cb(lv_event_t *e)
{
    (void)e;
    SettingDef *s = &settings[editor_setting_id];
    s->value += s->step;
    if (s->value > s->max_val) s->value = s->max_val;
    update_editor_value_label();
    if (!s->mega_backed) {
        save_setting_to_nvs(editor_setting_id);
        if (editor_setting_id == SETTING_BRIGHTNESS) apply_brightness();
        if (editor_setting_id == SETTING_LOGO_THEME) apply_theme();
    }
}

static void editor_done_cb(lv_event_t *e)
{
    (void)e;
    SettingDef *s = &settings[editor_setting_id];
    if (s->mega_backed) {
        send_set_command(editor_setting_id);
    }
    close_editor();
}

/* helper: create a plain clickable rectangle — avoids lv_btn theme bleed */
static lv_obj_t *make_plain_button(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                                   lv_color_t bg, lv_coord_t radius)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, w, h);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(obj, bg, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    return obj;
}

static void create_editor_panel(void)
{
    if (settings_editor_panel) return;

    settings_editor_panel = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(settings_editor_panel);
    lv_obj_set_size(settings_editor_panel, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(settings_editor_panel, 0, 0);
    lv_obj_set_style_bg_color(settings_editor_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(settings_editor_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(settings_editor_panel, 0, 0);
    lv_obj_set_style_pad_all(settings_editor_panel, 0, 0);
    lv_obj_clear_flag(settings_editor_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(settings_editor_panel, LV_OBJ_FLAG_CLICKABLE); /* block pass-through */

    editor_title_label = lv_label_create(settings_editor_panel);
    lv_obj_set_style_text_color(editor_title_label, lv_color_make(160, 160, 160), 0);
    lv_obj_set_style_text_font(editor_title_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(editor_title_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(editor_title_label, LCD_H_RES - 40);
    lv_obj_set_pos(editor_title_label, 20, 30);

    editor_value_label = lv_label_create(settings_editor_panel);
    lv_obj_set_style_text_color(editor_value_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(editor_value_label, &lv_font_montserrat_120, 0);
    lv_obj_set_style_text_align(editor_value_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(editor_value_label, LCD_H_RES - 40);
    lv_obj_set_pos(editor_value_label, 20, 120);

    /* Minus button */
    lv_obj_t *dec_btn = make_plain_button(settings_editor_panel, 160, 80,
                                          lv_color_make(60, 60, 60), 12);
    lv_obj_set_pos(dec_btn, 40, 320);
    lv_obj_add_event_cb(dec_btn, editor_dec_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *dec_lbl = lv_label_create(dec_btn);
    lv_label_set_text(dec_lbl, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(dec_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(dec_lbl, lv_color_white(), 0);
    lv_obj_center(dec_lbl);

    /* Done button */
    lv_obj_t *done_btn = make_plain_button(settings_editor_panel, 160, 80,
                                           lv_color_make(0, 120, 200), 12);
    lv_obj_align(done_btn, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_add_event_cb(done_btn, editor_done_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *done_lbl = lv_label_create(done_btn);
    lv_label_set_text(done_lbl, "DONE");
    lv_obj_set_style_text_font(done_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(done_lbl, lv_color_white(), 0);
    lv_obj_center(done_lbl);

    /* Plus button */
    lv_obj_t *inc_btn = make_plain_button(settings_editor_panel, 160, 80,
                                          lv_color_make(60, 60, 60), 12);
    lv_obj_set_pos(inc_btn, LCD_H_RES - 200, 320);
    lv_obj_add_event_cb(inc_btn, editor_inc_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *inc_lbl = lv_label_create(inc_btn);
    lv_label_set_text(inc_lbl, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(inc_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(inc_lbl, lv_color_white(), 0);
    lv_obj_center(inc_lbl);

    lv_obj_add_flag(settings_editor_panel, LV_OBJ_FLAG_HIDDEN);
}

static void open_editor(SettingId id)
{
    editor_setting_id = id;
    editor_visible = true;
    if (editor_title_label) {
        char title[64];
        snprintf(title, sizeof(title), "%s\n%s",
                 setting_group_names[settings[id].group], settings[id].name);
        lv_label_set_text(editor_title_label, title);
    }
    update_editor_value_label();
    if (settings_editor_panel) {
        lv_obj_clear_flag(settings_editor_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(settings_editor_panel);
    }
    if (settings_list_panel) lv_obj_add_flag(settings_list_panel, LV_OBJ_FLAG_HIDDEN);
}

static void close_editor(void)
{
    editor_visible = false;
    if (settings_editor_panel) lv_obj_add_flag(settings_editor_panel, LV_OBJ_FLAG_HIDDEN);
    if (settings_list_panel) {
        lv_obj_clear_flag(settings_list_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(settings_list_panel);
    }
}

/* ================================================================
 *  Settings UI — Grouped list
 * ================================================================ */

static void setting_row_click_cb(lv_event_t *e)
{
    SettingId id = (SettingId)(intptr_t)lv_event_get_user_data(e);
    open_editor(id);
}

static void reset_defaults_cb(lv_event_t *e)
{
    (void)e;
    reset_all_settings_to_defaults();
    apply_brightness();
    apply_theme();
    /* Send MEGA-backed defaults */
    for (int i = 0; i < SETTING_COUNT; i++) {
        if (settings[i].mega_backed) {
            send_set_command((SettingId)i);
        }
    }
    close_settings_menu();
}

static void create_settings_list(void)
{
    if (settings_list_panel) return;

    settings_list_panel = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(settings_list_panel);
    lv_obj_set_size(settings_list_panel, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(settings_list_panel, 0, 0);
    lv_obj_set_style_bg_color(settings_list_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(settings_list_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(settings_list_panel, 0, 0);
    lv_obj_set_style_pad_top(settings_list_panel, 10, 0);
    lv_obj_set_style_pad_bottom(settings_list_panel, 10, 0);
    lv_obj_set_style_pad_left(settings_list_panel, 20, 0);
    lv_obj_set_style_pad_right(settings_list_panel, 20, 0);
    lv_obj_set_flex_flow(settings_list_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(settings_list_panel, 4, 0);
    lv_obj_set_scrollbar_mode(settings_list_panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(settings_list_panel, LV_OBJ_FLAG_CLICKABLE);

    /* Title bar */
    lv_obj_t *title = lv_label_create(settings_list_panel);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_width(title, LCD_H_RES - 40);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    /* Build grouped rows */
    for (int g = 0; g < SGRP_COUNT; g++) {
        /* Group header */
        lv_obj_t *gh = lv_label_create(settings_list_panel);
        lv_label_set_text(gh, setting_group_names[g]);
        lv_obj_set_style_text_color(gh, lv_color_white(), 0);
        lv_obj_set_style_text_font(gh, &lv_font_montserrat_28, 0);
        lv_obj_set_width(gh, LCD_H_RES - 40);

        for (int i = 0; i < SETTING_COUNT; i++) {
            if (settings[i].group != (SettingGroup)g) continue;

            /* Row button */
            lv_obj_t *row = make_plain_button(settings_list_panel,
                                              LCD_H_RES - 60, 54,
                                              lv_color_make(35, 35, 35), 8);
            lv_obj_set_style_pad_left(row, 16, 0);
            lv_obj_set_style_pad_right(row, 16, 0);
            lv_obj_add_event_cb(row, setting_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t *name_lbl = lv_label_create(row);
            lv_label_set_text(name_lbl, settings[i].name);
            lv_obj_set_style_text_color(name_lbl, lv_color_make(220, 220, 220), 0);
            lv_obj_align(name_lbl, LV_ALIGN_LEFT_MID, 0, 0);

            lv_obj_t *val_lbl = lv_label_create(row);
            char buf[16];
            format_setting_value((SettingId)i, buf, sizeof(buf));
            lv_label_set_text(val_lbl, buf);
            lv_obj_set_style_text_color(val_lbl, lv_color_make(160, 160, 160), 0);
            lv_obj_align(val_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
        }
    }

    /* Reset defaults button */
    lv_obj_t *reset_btn = make_plain_button(settings_list_panel,
                                             LCD_H_RES - 60, 50,
                                             lv_color_make(140, 30, 30), 8);
    lv_obj_add_event_cb(reset_btn, reset_defaults_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *reset_lbl = lv_label_create(reset_btn);
    lv_label_set_text(reset_lbl, "RESET DEFAULTS");
    lv_obj_set_style_text_color(reset_lbl, lv_color_white(), 0);
    lv_obj_center(reset_lbl);

    lv_obj_add_flag(settings_list_panel, LV_OBJ_FLAG_HIDDEN);
}

/* ================================================================
 *  Settings open / close
 * ================================================================ */

static void open_settings_menu(void)
{
    settings_visible = true;
    editor_visible = false;

    /* refresh list values — rebuild is simplest for dynamic value labels */
    if (settings_list_panel) {
        lv_obj_del(settings_list_panel);
        settings_list_panel = NULL;
    }
    create_settings_list();

    lv_obj_clear_flag(settings_list_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(settings_list_panel);

    /* hide everything else */
    if (label_moco)   lv_obj_add_flag(label_moco,   LV_OBJ_FLAG_HIDDEN);
    if (label_jib)    lv_obj_add_flag(label_jib,    LV_OBJ_FLAG_HIDDEN);
    if (status_label) lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);
    set_drone_mode_visible(false);
}

static void close_settings_menu(void)
{
    settings_visible = false;
    editor_visible = false;
    if (settings_list_panel) lv_obj_add_flag(settings_list_panel, LV_OBJ_FLAG_HIDDEN);
    if (settings_editor_panel) lv_obj_add_flag(settings_editor_panel, LV_OBJ_FLAG_HIDDEN);

    /* restore display */
    if (current_display_mode == DISPLAY_MODE_MANUAL && !status_error_active && !mode_message_active) {
        if (label_moco) lv_obj_clear_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
        if (label_jib)  lv_obj_clear_flag(label_jib,  LV_OBJ_FLAG_HIDDEN);
    }
    if (current_display_mode == DISPLAY_MODE_DRONE) {
        set_drone_mode_visible(true);
    }
}

/* ================================================================
 *  Long-press detection timer
 * ================================================================ */

static void long_press_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (!touch_pressed || long_press_fired) return;

    int64_t elapsed_ms = (esp_timer_get_time() - touch_press_start_us) / 1000;
    if (elapsed_ms < LONG_PRESS_MS) return;

    long_press_fired = true;

    if (editor_visible) {
        /* long-press in editor → close entire menu */
        close_editor();
        close_settings_menu();
    } else if (settings_visible) {
        close_settings_menu();
    } else {
        open_settings_menu();
    }
}

static void show_status_on_display(const char *msg, bool is_error)
{
    status_error_active = is_error;

    if (is_error) {
        if (label_moco) {
            lv_obj_add_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
        }
        if (label_jib) {
            lv_obj_add_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
        }

        if (status_label == NULL) {
            status_label = lv_label_create(lv_scr_act());
            lv_obj_set_style_text_color(status_label, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_text_font(status_label, &lv_font_montserrat_48, LV_PART_MAIN);
            lv_obj_set_style_text_line_space(status_label, 8, LV_PART_MAIN);
            lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
            lv_label_set_long_mode(status_label, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(status_label, LCD_H_RES - 40);
        }

        lv_label_set_text(status_label, msg);
        lv_obj_center(status_label);
        lv_obj_clear_flag(status_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (status_label) {
        lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (!settings_visible) {
        if (label_moco) {
            lv_obj_clear_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
        }
        if (label_jib) {
            lv_obj_clear_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void show_mode_message_on_display(const char *mode_msg, uint64_t now_ms)
{
    if (mode_msg == NULL || mode_msg[0] == '\0') {
        mode_message_active = false;
        show_status_on_display(NULL, false);
        return;
    }

    status_error_active = false;
    mode_message_active = true;
    mode_message_until_ms = now_ms + MODE_MESSAGE_DURATION_MS;

    if (label_moco) {
        lv_obj_add_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
    }
    if (label_jib) {
        lv_obj_add_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
    }

    if (status_label == NULL) {
        status_label = lv_label_create(lv_scr_act());
        lv_obj_set_style_text_color(status_label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(status_label, &lv_font_montserrat_48, LV_PART_MAIN);
        lv_obj_set_style_text_line_space(status_label, 8, LV_PART_MAIN);
        lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_long_mode(status_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(status_label, LCD_H_RES - 40);
    }

    char mode_text[96];
    snprintf(mode_text, sizeof(mode_text), "MODE\n%s", mode_msg);
    lv_label_set_text(status_label, mode_text);
    lv_obj_center(status_label);
    lv_obj_clear_flag(status_label, LV_OBJ_FLAG_HIDDEN);
}

static void show_temporary_message_on_display(const char *msg, uint64_t now_ms)
{
    if (msg == NULL || msg[0] == '\0') {
        mode_message_active = false;
        show_status_on_display(NULL, false);
        return;
    }

    status_error_active = false;
    mode_message_active = true;
    mode_message_until_ms = now_ms + MODE_MESSAGE_DURATION_MS;

    if (label_moco) {
        lv_obj_add_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
    }
    if (label_jib) {
        lv_obj_add_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
    }

    if (status_label == NULL) {
        status_label = lv_label_create(lv_scr_act());
        lv_obj_set_style_text_color(status_label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(status_label, &lv_font_montserrat_48, LV_PART_MAIN);
        lv_obj_set_style_text_line_space(status_label, 8, LV_PART_MAIN);
        lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_long_mode(status_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(status_label, LCD_H_RES - 40);
    }

    lv_label_set_text(status_label, msg);
    lv_obj_center(status_label);
    lv_obj_clear_flag(status_label, LV_OBJ_FLAG_HIDDEN);
}

static void show_timelapse_interval_on_display(int interval_seconds, uint64_t now_ms)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "TIMELAPSE\nINTERVAL\n%ds", interval_seconds);
    mode_to_restore_after_message = current_display_mode;
    restore_mode_after_message = true;
    show_temporary_message_on_display(msg, now_ms);
    mode_message_until_ms = now_ms + TIMELAPSE_MESSAGE_DURATION_MS;
}

static void show_timelapse_stepdist_on_display(int stepdist_ms, uint64_t now_ms)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "TIMELAPSE\nSTEP DIST\n%dms", stepdist_ms);
    mode_to_restore_after_message = current_display_mode;
    restore_mode_after_message = true;
    show_temporary_message_on_display(msg, now_ms);
    mode_message_until_ms = now_ms + TIMELAPSE_MESSAGE_DURATION_MS;
}

static void update_drone_lift_indicator(void)
{
    if (drone_left_stick == NULL) {
        return;
    }

    lv_coord_t x = DRONE_LEFT_STICK_BASE_X + (drone_swing_display_percent * DRONE_HORIZ_INDICATOR_OFFSET) / 100;
    lv_coord_t y = DRONE_LEFT_STICK_BASE_Y - (drone_lift_display_percent * DRONE_LIFT_INDICATOR_OFFSET) / 100;

    lv_obj_set_pos(drone_left_stick, x, y);
}

static void update_drone_tilt_indicator(void)
{
    if (drone_right_stick == NULL) {
        return;
    }

    lv_coord_t x = DRONE_RIGHT_STICK_BASE_X + (drone_pan_display_percent * DRONE_HORIZ_INDICATOR_OFFSET) / 100;
    lv_coord_t y = DRONE_RIGHT_STICK_BASE_Y - (drone_tilt_display_percent * DRONE_LIFT_INDICATOR_OFFSET) / 100;

    lv_obj_set_pos(drone_right_stick, x, y);
}

static int clamp_display_percent(int value)
{
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return value;
}

static void update_drone_stick_colors(void)
{
    if (drone_left_stick) {
        lv_obj_set_style_bg_color(drone_left_stick,
                                  drone_left_stick_pressed ? lv_color_make(0, 255, 0) : lv_color_white(),
                                  LV_PART_MAIN);
    }
    if (drone_right_stick) {
        lv_obj_set_style_bg_color(drone_right_stick,
                                  drone_right_stick_pressed ? lv_color_make(0, 255, 0) : lv_color_white(),
                                  LV_PART_MAIN);
    }
}

static void set_flowlapse_progress(int progress_percent)
{
    if (progress_percent < 0) {
        progress_percent = 0;
    } else if (progress_percent > 100) {
        progress_percent = 100;
    }

    if (progress_percent >= 99) {
        progress_percent = 100;
    }
    
    flowlapse_progress_percent = progress_percent;
    
    if (drone_flowlapse_fill && drone_flowlapse_bar) {
        int bar_width = lv_obj_get_width(drone_flowlapse_bar);
        int bar_height = lv_obj_get_height(drone_flowlapse_bar);
        int fill_width = ((bar_width - 6) * progress_percent) / 100;
        int fill_height = bar_height - 6;
        if (fill_height < 2) {
            fill_height = 2;
        }
        lv_obj_set_width(drone_flowlapse_fill, fill_width);
        lv_obj_set_height(drone_flowlapse_fill, fill_height);
    }
}

static void update_flowlapse_layout(void)
{
    if (!drone_flowlapse_bar || !drone_flowlapse_label) {
        return;
    }

    int bar_x = FLOWLAPSE_BAR_NORMAL_X;
    int bar_y = FLOWLAPSE_BAR_NORMAL_Y;
    int bar_w = FLOWLAPSE_BAR_NORMAL_W;
    int bar_h = FLOWLAPSE_BAR_NORMAL_H;
    int label_y = FLOWLAPSE_LABEL_NORMAL_Y;

    if (flowlapse_active) {
        bar_x = FLOWLAPSE_BAR_PLAYBACK_X;
        bar_y = FLOWLAPSE_BAR_PLAYBACK_Y;
        bar_w = FLOWLAPSE_BAR_PLAYBACK_W;
        bar_h = FLOWLAPSE_BAR_PLAYBACK_H;
        label_y = FLOWLAPSE_LABEL_PLAYBACK_Y;
    }

    lv_obj_set_pos(drone_flowlapse_bar, bar_x, bar_y);
    lv_obj_set_size(drone_flowlapse_bar, bar_w, bar_h);
    lv_obj_set_pos(drone_flowlapse_label, bar_x, label_y);
    lv_obj_set_width(drone_flowlapse_label, bar_w);

    set_flowlapse_progress(flowlapse_progress_percent);
}

static void refresh_flowlapse_status_label(void)
{
    if (!drone_flowlapse_label) {
        return;
    }

    bool show_l2_boost = flowlapse_active && drone_precision_modifier_active;
    if (show_l2_boost) {
        char boosted_status[80];
        snprintf(boosted_status, sizeof(boosted_status), "%s | L2 BOOST", flowlapse_status_text);
        lv_label_set_text(drone_flowlapse_label, boosted_status);
    } else {
        lv_label_set_text(drone_flowlapse_label, flowlapse_status_text);
    }
}

static void set_flowlapse_status(bool active, const char *text)
{
    flowlapse_active = active;

    if (!active) {
        flowlapse_playback_active = false;
    }

    if (text && text[0] != '\0') {
        snprintf(flowlapse_status_text, sizeof(flowlapse_status_text), "%s", text);
    }

    refresh_flowlapse_status_label();

    bool show = active && current_display_mode == DISPLAY_MODE_DRONE;
    bool show_joystick_cluster = (current_display_mode == DISPLAY_MODE_DRONE) && !flowlapse_active;
    bool show_modifiers = (current_display_mode == DISPLAY_MODE_DRONE) && !flowlapse_active;

    if (drone_precision_label) {
        show_modifiers ? lv_obj_clear_flag(drone_precision_label, LV_OBJ_FLAG_HIDDEN)
                       : lv_obj_add_flag(drone_precision_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_boost_label) {
        show_modifiers ? lv_obj_clear_flag(drone_boost_label, LV_OBJ_FLAG_HIDDEN)
                       : lv_obj_add_flag(drone_boost_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_precision_state_box) {
        show_modifiers ? lv_obj_clear_flag(drone_precision_state_box, LV_OBJ_FLAG_HIDDEN)
                       : lv_obj_add_flag(drone_precision_state_box, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_boost_state_box) {
        show_modifiers ? lv_obj_clear_flag(drone_boost_state_box, LV_OBJ_FLAG_HIDDEN)
                       : lv_obj_add_flag(drone_boost_state_box, LV_OBJ_FLAG_HIDDEN);
    }

    if (drone_left_ring) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_left_ring, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(drone_left_ring, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_right_ring) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_right_ring, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(drone_right_ring, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_left_stick) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_left_stick, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(drone_left_stick, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_right_stick) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_right_stick, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(drone_right_stick, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_center_line) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_center_line, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(drone_center_line, LV_OBJ_FLAG_HIDDEN);
    }

    update_flowlapse_layout();

    if (drone_flowlapse_bar) {
        show ? lv_obj_clear_flag(drone_flowlapse_bar, LV_OBJ_FLAG_HIDDEN)
             : lv_obj_add_flag(drone_flowlapse_bar, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_flowlapse_label) {
        show ? lv_obj_clear_flag(drone_flowlapse_label, LV_OBJ_FLAG_HIDDEN)
             : lv_obj_add_flag(drone_flowlapse_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_flowlapse_waypoint_label) {
        (show && flowlapse_waypoint_indicator_active)
            ? lv_obj_clear_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN)
            : lv_obj_add_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN);
    }
}

static void set_flowlapse_waypoint_count(int current, int total)
{
    flowlapse_waypoint_current = current;
    flowlapse_waypoint_total = total;

    if (drone_flowlapse_waypoint_label) {
        char waypoint_text[16];
        snprintf(waypoint_text, sizeof(waypoint_text), "%d/%d", current, total);
        lv_label_set_text(drone_flowlapse_waypoint_label, waypoint_text);
    }
}

static void set_drone_modifier_indicator(bool precision_active, bool boost_active)
{
    if (precision_active) {
        boost_active = false;
    }

    drone_precision_modifier_active = precision_active;
    drone_boost_modifier_active = boost_active;

    if (drone_precision_label) {
        lv_label_set_text(drone_precision_label, "PRECISION");
    }

    if (drone_boost_label) {
        lv_label_set_text(drone_boost_label, "BOOST");
    }

    if (drone_precision_state_box) {
        lv_obj_set_style_bg_color(drone_precision_state_box,
                                  precision_active ? lv_color_make(0, 0, 255) : lv_color_make(120, 120, 120),
                                  LV_PART_MAIN);
    }

    if (drone_boost_state_box) {
        lv_obj_set_style_bg_color(drone_boost_state_box,
                                  boost_active ? lv_color_make(0, 0, 255) : lv_color_make(120, 120, 120),
                                  LV_PART_MAIN);
    }

    if (drone_precision_label) {
        lv_obj_set_style_text_color(drone_precision_label, lv_color_white(), LV_PART_MAIN);
    }
    if (drone_boost_label) {
        lv_obj_set_style_text_color(drone_boost_label, lv_color_white(), LV_PART_MAIN);
    }
}

static bool parse_drone_stick_payload(const char *payload, int *swing_dir, int *lift_dir, int *pan_dir, int *tilt_dir,
                                      bool *left_stick_click, bool *right_stick_click)
{
    if (payload == NULL || swing_dir == NULL || lift_dir == NULL || pan_dir == NULL || tilt_dir == NULL
        || left_stick_click == NULL || right_stick_click == NULL) {
        return false;
    }

    int swing = 0;
    int lift = 0;
    int pan = 0;
    int tilt = 0;
    int left_click = 0;
    int right_click = 0;

    int parsed = sscanf(payload, "%d,%d,%d,%d,%d,%d",
                        &swing, &lift, &pan, &tilt, &left_click, &right_click);
    if (parsed < 4) {
        return false;
    }

    *swing_dir = swing;
    *lift_dir = lift;
    *pan_dir = pan;
    *tilt_dir = tilt;
    *left_stick_click = (parsed >= 5) ? (left_click != 0) : false;
    *right_stick_click = (parsed >= 6) ? (right_click != 0) : false;
    return true;
}

static const char *drone_horiz_dir_to_text(int dir)
{
    if (dir > 0) {
        return "RIGHT";
    }
    if (dir < 0) {
        return "LEFT";
    }
    return "CENTER";
}

static const char *drone_vert_dir_to_text(int dir)
{
    if (dir > 0) {
        return "UP";
    }
    if (dir < 0) {
        return "DOWN";
    }
    return "CENTER";
}

static void set_drone_mode_visible(bool visible)
{
    bool show_joystick_cluster = visible && !flowlapse_active;
    bool show_modifiers = visible && !flowlapse_active;

    if (visible) {
        if (label_moco) {
            lv_obj_add_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
        }
        if (label_jib) {
            lv_obj_add_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
        }
        if (status_label) {
            lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (drone_title_label) {
        visible ? lv_obj_clear_flag(drone_title_label, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_title_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_subtitle_label) {
        visible ? lv_obj_clear_flag(drone_subtitle_label, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_subtitle_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_left_ring) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_left_ring, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_left_ring, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_right_ring) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_right_ring, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_right_ring, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_left_stick) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_left_stick, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_left_stick, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_right_stick) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_right_stick, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_right_stick, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_center_line) {
        show_joystick_cluster ? lv_obj_clear_flag(drone_center_line, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_center_line, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_precision_label) {
        show_modifiers ? lv_obj_clear_flag(drone_precision_label, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_precision_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_boost_label) {
        show_modifiers ? lv_obj_clear_flag(drone_boost_label, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_boost_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_precision_state_box) {
        show_modifiers ? lv_obj_clear_flag(drone_precision_state_box, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_precision_state_box, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_boost_state_box) {
        show_modifiers ? lv_obj_clear_flag(drone_boost_state_box, LV_OBJ_FLAG_HIDDEN) : lv_obj_add_flag(drone_boost_state_box, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_flowlapse_bar) {
        (visible && flowlapse_active) ? lv_obj_clear_flag(drone_flowlapse_bar, LV_OBJ_FLAG_HIDDEN)
                                      : lv_obj_add_flag(drone_flowlapse_bar, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_flowlapse_label) {
        (visible && flowlapse_active) ? lv_obj_clear_flag(drone_flowlapse_label, LV_OBJ_FLAG_HIDDEN)
                                      : lv_obj_add_flag(drone_flowlapse_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (drone_flowlapse_waypoint_label) {
        (visible && flowlapse_waypoint_indicator_active) ? lv_obj_clear_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN)
                                                         : lv_obj_add_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN);
    }

    if (visible) {
        update_flowlapse_layout();
    }
}

static const char *display_mode_to_text(DisplayMode mode)
{
    switch (mode) {
    case DISPLAY_MODE_MANUAL:
        return "MANUAL";
    case DISPLAY_MODE_DRONE:
        return "DRONE";
    case DISPLAY_MODE_TIMELAPSE:
        return "TIMELAPSE";
    case DISPLAY_MODE_BOUNCE:
        return "BOUNCE";
    case DISPLAY_MODE_ERROR:
    default:
        return "ERROR";
    }
}

static void switch_display_mode(DisplayMode mode, const char *detail_msg, uint64_t now_ms)
{
    DisplayMode previous_mode = current_display_mode;
    current_display_mode = mode;
    set_drone_mode_visible(false);

    if (mode == DISPLAY_MODE_ERROR) {
        mode_message_active = false;
        restore_mode_after_message = false;
        set_flowlapse_status(false, NULL);
        show_status_on_display((detail_msg && detail_msg[0]) ? detail_msg : "NO CONTROLLER\nFOUND", true);
        return;
    }

    last_non_error_mode = mode;
    status_error_active = false;

    if (mode == DISPLAY_MODE_MANUAL) {
        mode_message_active = false;
        restore_mode_after_message = false;
        set_flowlapse_status(false, NULL);
        show_status_on_display(NULL, false);
        return;
    }

    if (mode == DISPLAY_MODE_DRONE) {
        mode_message_active = false;
        restore_mode_after_message = false;
        status_error_active = false;
        if (previous_mode != DISPLAY_MODE_DRONE) {
            drone_swing_display_state = DRONE_HORIZ_NEUTRAL;
            drone_lift_display_state = DRONE_LIFT_NEUTRAL;
            drone_pan_display_state = DRONE_HORIZ_NEUTRAL;
            drone_tilt_display_state = DRONE_TILT_NEUTRAL;
            drone_swing_display_percent = 0;
            drone_lift_display_percent = 0;
            drone_pan_display_percent = 0;
            drone_tilt_display_percent = 0;
            drone_left_stick_pressed = false;
            drone_right_stick_pressed = false;
            drone_swing_pulse_start_ms = 0;
            drone_lift_pulse_start_ms = 0;
            drone_pan_pulse_start_ms = 0;
            drone_tilt_pulse_start_ms = 0;
            update_drone_lift_indicator();
            update_drone_tilt_indicator();
            update_drone_stick_colors();
            set_flowlapse_status(false, "FLOWLAPSE READY");
            set_flowlapse_progress(0);
        }
        set_drone_mode_visible(true);
        return;
    }

    set_flowlapse_status(false, NULL);
    restore_mode_after_message = false;
    show_mode_message_on_display(display_mode_to_text(mode), now_ms);
}

static void status_uart_task(void *arg)
{
    (void)arg;
    uint8_t byte = 0;
    char line[256];
    size_t line_len = 0;
    uint64_t last_status_rx_ms = esp_timer_get_time() / 1000ULL;

    while (1) {
        uint64_t now_ms = esp_timer_get_time() / 1000ULL;
        if (status_error_active && (now_ms - last_status_rx_ms > STATUS_SIGNAL_TIMEOUT_MS)) {
            if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                mode_message_active = false;
                show_status_on_display(NULL, false);
                xSemaphoreGive(lvgl_mux);
            }
            last_status_rx_ms = now_ms;
        }

        if (mode_message_active && now_ms >= mode_message_until_ms) {
            if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                mode_message_active = false;
                if (restore_mode_after_message) {
                    restore_mode_after_message = false;
                    switch_display_mode(mode_to_restore_after_message, NULL, now_ms);
                } else {
                    show_status_on_display(NULL, false);
                }
                xSemaphoreGive(lvgl_mux);
            }
        }

        int len = uart_read_bytes(STATUS_UART_PORT, &byte, 1, pdMS_TO_TICKS(5));
        if (len <= 0) {
            continue;
        }

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            if (line_len == 0) {
                continue;
            }

            line[line_len] = '\0';
            if (strncmp(line, "DRONE_STICK:", 12) != 0) {
                ESP_LOGI(TAG, "Mega status: %s", line);
            }
            last_status_rx_ms = now_ms;

            if (strncmp(line, "EMERGENCY_STOP:ACTIVE", 21) == 0
                || (strncmp(line, "EMERGENCY STOP", 14) == 0 && strstr(line, "RELEASED") == NULL)) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    emergency_stop_active = true;
                    switch_display_mode(DISPLAY_MODE_ERROR, "EMERGENCY\nSTOP", now_ms);
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "EMERGENCY_STOP:RELEASED", 23) == 0
                       || strncmp(line, "EMERGENCY STOP RELEASED", 23) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    emergency_stop_active = false;
                    mode_to_restore_after_message = controller_connected ? last_non_error_mode : DISPLAY_MODE_ERROR;
                    restore_mode_after_message = true;
                    show_temporary_message_on_display("EMERGENCY STOP\nRELEASED", now_ms);
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "CONTROLLER_ERROR:", 17) == 0) {
                const char *msg = line + 17;
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    controller_connected = false;
                    if (!emergency_stop_active) {
                        switch_display_mode(DISPLAY_MODE_ERROR, msg, now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "CONTROLLER_OK:", 14) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    controller_connected = true;
                    if (!emergency_stop_active && !mode_message_active) {
                        switch_display_mode(last_non_error_mode, NULL, now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "TIMELAPSE_INTERVAL:", 19) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int interval_seconds = 0;
                    if (!emergency_stop_active && sscanf(line, "TIMELAPSE_INTERVAL:%d", &interval_seconds) == 1) {
                        settings[SETTING_TL_INTERVAL].value = interval_seconds;
                        show_timelapse_interval_on_display(interval_seconds, now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "TIMELAPSE_STEPDIST:", 19) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int stepdist_ms = 0;
                    if (!emergency_stop_active && sscanf(line, "TIMELAPSE_STEPDIST:%d", &stepdist_ms) == 1) {
                        settings[SETTING_TL_STEPDIST].value = stepdist_ms;
                        show_timelapse_stepdist_on_display(stepdist_ms, now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "RUMBLE_MUTE:ON", 14) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    settings[SETTING_RUMBLE_MUTE].value = 1;
                    if (!emergency_stop_active) {
                        mode_to_restore_after_message = current_display_mode;
                        restore_mode_after_message = true;
                        show_temporary_message_on_display("RUMBLE\nMUTE ON", now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "RUMBLE_MUTE:OFF", 15) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    settings[SETTING_RUMBLE_MUTE].value = 0;
                    if (!emergency_stop_active) {
                        mode_to_restore_after_message = current_display_mode;
                        restore_mode_after_message = true;
                        show_temporary_message_on_display("RUMBLE\nMUTE OFF", now_ms);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "CONTROL:", 8) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    if (!emergency_stop_active) {
                        const char *control = line + 8;
                        const char *msg = NULL;
                        if (strcmp(control, "FOCUS_LEFT") == 0) {
                            msg = "FOCUS LEFT";
                        } else if (strcmp(control, "FOCUS_RIGHT") == 0) {
                            msg = "FOCUS RIGHT";
                        } else if (strcmp(control, "FOCUS_SPEED_DOWN") == 0) {
                            msg = "FOCUS SPEED\nDOWN";
                        } else if (strcmp(control, "FOCUS_SPEED_UP") == 0) {
                            msg = "FOCUS SPEED\nUP";
                        } else if (strcmp(control, "L1_PAN_SWING_UP") == 0) {
                            msg = "PAN+SWING SPEED\nUP";
                        } else if (strcmp(control, "L2_PAN_SWING_DOWN") == 0) {
                            msg = "PAN+SWING SPEED\nDOWN";
                        } else if (strcmp(control, "R1_LIFT_TILT_UP") == 0) {
                            msg = "LIFT+TILT SPEED\nUP";
                        } else if (strcmp(control, "R2_LIFT_TILT_DOWN") == 0) {
                            msg = "LIFT+TILT SPEED\nDOWN";
                        } else if (strcmp(control, "SWING_SOLO_LEFT") == 0) {
                            msg = "SWING SOLO\nLEFT";
                        } else if (strcmp(control, "SWING_SOLO_RIGHT") == 0) {
                            msg = "SWING SOLO\nRIGHT";
                        } else if (strcmp(control, "PAN_SOLO_LEFT") == 0) {
                            msg = "PAN SOLO\nLEFT";
                        } else if (strcmp(control, "PAN_SOLO_RIGHT") == 0) {
                            msg = "PAN SOLO\nRIGHT";
                        } else if (strcmp(control, "SWING_PAN_LEFT") == 0) {
                            msg = "SWING+PAN\nLEFT";
                        } else if (strcmp(control, "SWING_PAN_RIGHT") == 0) {
                            msg = "SWING+PAN\nRIGHT";
                        } else if (strcmp(control, "LIFT_SOLO_UP") == 0) {
                            msg = "LIFT SOLO\nUP";
                        } else if (strcmp(control, "LIFT_SOLO_DOWN") == 0) {
                            msg = "LIFT SOLO\nDOWN";
                        } else if (strcmp(control, "LIFT_TILT_UP") == 0) {
                            msg = "LIFT+TILT\nUP";
                        } else if (strcmp(control, "LIFT_TILT_DOWN") == 0) {
                            msg = "LIFT+TILT\nDOWN";
                        } else if (strcmp(control, "TILT_SOLO_UP") == 0) {
                            msg = "TILT SOLO\nUP";
                        } else if (strcmp(control, "TILT_SOLO_DOWN") == 0) {
                            msg = "TILT SOLO\nDOWN";
                        } else if (strcmp(control, "L3_WAYPOINT_RECORD") == 0) {
                            int next_current = flowlapse_waypoint_current + 1;
                            int total = flowlapse_waypoint_total;
                            if (total <= 0) {
                                total = 8;
                            }
                            if (next_current > total) {
                                next_current = total;
                            }
                            flowlapse_waypoint_indicator_active = true;
                            set_flowlapse_waypoint_count(next_current, total);
                        } else if (strcmp(control, "L3_BOUNCE_ENDPOINT") == 0) {
                            msg = "L3\nBOUNCE\nENDPOINT SET";
                        }

                        if (msg != NULL) {
                            mode_to_restore_after_message = current_display_mode;
                            restore_mode_after_message = true;
                            show_temporary_message_on_display(msg, now_ms);
                        }
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "MODE:") != NULL) {
                const char *mode_msg = strstr(line, "MODE:") + 5;
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    if (!emergency_stop_active && !mode_message_active) {
                        if (strncmp(mode_msg, "MANUAL", 6) == 0) {
                            switch_display_mode(DISPLAY_MODE_MANUAL, NULL, now_ms);
                        } else if (strncmp(mode_msg, "DRONE", 5) == 0) {
                            switch_display_mode(DISPLAY_MODE_DRONE, NULL, now_ms);
                        } else if (strncmp(mode_msg, "TIMELAPSE", 9) == 0) {
                            switch_display_mode(DISPLAY_MODE_TIMELAPSE, NULL, now_ms);
                        } else if (strncmp(mode_msg, "BOUNCE", 6) == 0) {
                            switch_display_mode(DISPLAY_MODE_BOUNCE, NULL, now_ms);
                        } else {
                            switch_display_mode(DISPLAY_MODE_MANUAL, NULL, now_ms);
                        }
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "DRONE MODE DEACTIVATED") != NULL) {
                                    flowlapse_waypoint_indicator_active = false;
                flowlapse_playback_active = false;
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    set_flowlapse_status(false, "FLOWLAPSE READY");
                    set_flowlapse_progress(0);
                    set_flowlapse_waypoint_count(0, 0);
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "DRONE MODE ACTIVATED") != NULL) {
                                    flowlapse_waypoint_indicator_active = false;
                flowlapse_playback_active = false;
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    switch_display_mode(DISPLAY_MODE_DRONE, NULL, now_ms);
                    set_flowlapse_status(false, "FLOWLAPSE READY");
                    set_flowlapse_progress(0);
                    set_flowlapse_waypoint_count(0, 0);
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "progress=") != NULL && strstr(line, "eta=") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int percent = 0;
                    int eta_sec = 0;
                    int waypoint_current = 0;
                    int waypoint_total = 0;
                    char *progress_ptr = strstr(line, "progress=");
                    char *eta_ptr = strstr(line, "eta=");
                    char *waypoint_ptr = strstr(line, "waypoint ");
                    
                    if (progress_ptr && eta_ptr && sscanf(progress_ptr, "progress=%d%%", &percent) == 1 && sscanf(eta_ptr, "eta=%ds", &eta_sec) == 1) {
                        flowlapse_playback_active = true;
                        set_flowlapse_progress(percent);
                        char status_buf[48];
                        snprintf(status_buf, sizeof(status_buf), "%d%% ETA %ds", percent, eta_sec);
                        set_flowlapse_status(true, status_buf);
                        
                        if (waypoint_ptr && sscanf(waypoint_ptr, "waypoint %d/%d", &waypoint_current, &waypoint_total) == 2) {
                            ESP_LOGI(TAG, "Parsed waypoint: %d/%d from line", waypoint_current, waypoint_total);
                            set_flowlapse_waypoint_count(waypoint_current, waypoint_total);
                        } else {
                            ESP_LOGI(TAG, "Failed to parse waypoint from: %s", waypoint_ptr ? waypoint_ptr : "NULL");
                        }
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "WAYPOINT_COUNT:") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int waypoint_current = 0;
                    int waypoint_total = 0;
                    if (sscanf(line, "WAYPOINT_COUNT:%d/%d", &waypoint_current, &waypoint_total) == 2) {
                        set_flowlapse_waypoint_count(waypoint_current, waypoint_total);
                        flowlapse_waypoint_indicator_active = (waypoint_current > 0);
                        if (!flowlapse_waypoint_indicator_active) {
                            set_flowlapse_status(false, "FLOWLAPSE READY");
                        }
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "PREVIEW_WAYPOINT:") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int waypoint_current = 0;
                    int waypoint_total = 0;
                    if (sscanf(line, "PREVIEW_WAYPOINT:%d/%d", &waypoint_current, &waypoint_total) == 2) {
                        flowlapse_waypoint_indicator_active = true;
                        set_flowlapse_waypoint_count(waypoint_current, waypoint_total);
                        set_flowlapse_status(true, "FLOWLAPSE PREVIEW");
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "DRONE_MODIFIER:") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int precision = 0;
                    int boost = 0;
                    char *modifier_ptr = strstr(line, "DRONE_MODIFIER:");
                    if (modifier_ptr != NULL
                        && sscanf(modifier_ptr, "DRONE_MODIFIER:precision=%d,boost=%d", &precision, &boost) == 2) {
                        bool precision_active = (precision != 0);
                        bool boost_active = (boost != 0);
                        set_drone_modifier_indicator(precision_active, boost_active);
                        refresh_flowlapse_status_label();
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "waypoint recorded") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int waypoint_current = 0;
                    int waypoint_total = 0;
                    if (sscanf(line, "Flowlapse: waypoint recorded %d/%d", &waypoint_current, &waypoint_total) == 2) {
                        flowlapse_waypoint_indicator_active = true;
                        set_flowlapse_waypoint_count(waypoint_current, waypoint_total);
                        if (current_display_mode == DISPLAY_MODE_DRONE) {
                            lv_obj_clear_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "Flowlapse:") != NULL
                       || strstr(line, "recording armed") != NULL
                       || strstr(line, "preview started") != NULL
                       || strstr(line, "capture run started") != NULL
                       || strstr(line, "capture paused") != NULL
                       || strstr(line, "capture resumed") != NULL
                       || strstr(line, "capture complete") != NULL
                       || strstr(line, "canceled") != NULL) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    int percent = 0;
                    int eta_sec = 0;
                    if (sscanf(line, "Flowlapse: %d%% ETA %ds", &percent, &eta_sec) == 2) {
                        flowlapse_playback_active = true;
                        set_flowlapse_progress(percent);
                        char status_buf[48];
                        snprintf(status_buf, sizeof(status_buf), "%d%% ETA %ds", percent, eta_sec);
                        set_flowlapse_status(true, status_buf);
                    } else if (strstr(line, "recording stopped. Press SELECT again for preview") != NULL) {
                        flowlapse_playback_active = false;
                        set_flowlapse_status(true, "FLOWLAPSE READY");
                    } else if (strstr(line, "recording armed") != NULL) {
                        flowlapse_playback_active = false;
                        set_flowlapse_status(true, "FLOWLAPSE READY");
                    } else if (strstr(line, "full course wiped") != NULL
                               || strstr(line, "course cleared") != NULL
                               || strstr(line, "recording re-armed") != NULL) {
                        flowlapse_playback_active = false;
                        flowlapse_waypoint_indicator_active = false;
                        set_flowlapse_waypoint_count(0, 0);
                        set_flowlapse_status(false, "FLOWLAPSE READY");
                    } else if (strstr(line, "preview started") != NULL) {
                        flowlapse_playback_active = true;
                        flowlapse_waypoint_indicator_active = true;
                        set_flowlapse_status(true, "FLOWLAPSE PREVIEW");
                    } else if (strstr(line, "preview complete") != NULL) {
                        flowlapse_playback_active = false;
                        set_flowlapse_status(true, "PREVIEW COMPLETE press START to run capture");
                    } else if (strstr(line, "capture run started") != NULL) {
                        flowlapse_playback_active = true;
                        set_flowlapse_status(true, "FLOWLAPSE CAPTURE");
                        set_flowlapse_progress(0);
                    } else if (strstr(line, "capture paused") != NULL) {
                        flowlapse_playback_active = true;
                        set_flowlapse_status(true, "FLOWLAPSE PAUSED");
                    } else if (strstr(line, "capture resumed") != NULL) {
                        flowlapse_playback_active = true;
                        set_flowlapse_status(true, "FLOWLAPSE CAPTURE");
                    } else if (strstr(line, "capture complete") != NULL) {
                        flowlapse_playback_active = false;
                        flowlapse_waypoint_indicator_active = false;
                        set_flowlapse_progress(100);
                        switch_display_mode(DISPLAY_MODE_DRONE, NULL, now_ms);
                        set_flowlapse_status(false, "FLOWLAPSE READY");
                        set_flowlapse_progress(0);
                        set_flowlapse_waypoint_count(0, 0);
                        set_drone_mode_visible(true);
                    } else if (strstr(line, "canceled") != NULL) {
                        flowlapse_playback_active = false;
                        flowlapse_waypoint_indicator_active = false;
                        switch_display_mode(DISPLAY_MODE_DRONE, NULL, now_ms);
                        set_flowlapse_status(false, "FLOWLAPSE READY");
                        set_flowlapse_progress(0);
                        set_flowlapse_waypoint_count(0, 0);
                        set_drone_mode_visible(true);
                    }
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strstr(line, "DRONE_STICK:") != NULL) {
                const char *drone_stick_payload = strstr(line, "DRONE_STICK:") + 12;
                int swing_value = 0;
                int lift_value = 0;
                int pan_value = 0;
                int tilt_value = 0;
                bool left_stick_click = false;
                bool right_stick_click = false;
                if (parse_drone_stick_payload(drone_stick_payload, &swing_value, &lift_value, &pan_value, &tilt_value,
                                              &left_stick_click, &right_stick_click)) {
                    swing_value = clamp_display_percent(swing_value);
                    lift_value = clamp_display_percent(lift_value);
                    pan_value = clamp_display_percent(pan_value);
                    tilt_value = clamp_display_percent(tilt_value);

                    int swing_dir = (swing_value > 0) - (swing_value < 0);
                    int lift_dir = (lift_value > 0) - (lift_value < 0);
                    int pan_dir = (pan_value > 0) - (pan_value < 0);
                    int tilt_dir = (tilt_value > 0) - (tilt_value < 0);

                    if (swing_dir != last_logged_swing_dir
                        || lift_dir != last_logged_lift_dir
                        || pan_dir != last_logged_pan_dir
                        || tilt_dir != last_logged_tilt_dir) {
                        ESP_LOGI(TAG, "DRONE dir | swing=%s lift=%s pan=%s tilt=%s",
                                 drone_horiz_dir_to_text(swing_dir),
                                 drone_vert_dir_to_text(lift_dir),
                                 drone_horiz_dir_to_text(pan_dir),
                                 drone_vert_dir_to_text(tilt_dir));
                        last_logged_swing_dir = swing_dir;
                        last_logged_lift_dir = lift_dir;
                        last_logged_pan_dir = pan_dir;
                        last_logged_tilt_dir = tilt_dir;
                    }

                    if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                        if (!emergency_stop_active && current_display_mode != DISPLAY_MODE_DRONE) {
                            switch_display_mode(DISPLAY_MODE_DRONE, NULL, now_ms);
                        }
                        if (flowlapse_active) {
                            set_flowlapse_status(false, "FLOWLAPSE READY");
                            set_flowlapse_progress(0);
                        }

                        drone_left_stick_pressed = left_stick_click;
                        drone_right_stick_pressed = right_stick_click;

                        if (swing_value != 0) {
                            if (drone_swing_display_percent == 0) {
                                drone_swing_pulse_start_ms = now_ms;
                            }
                            drone_swing_display_percent = swing_value;
                        } else if (drone_swing_display_percent != 0
                                   && now_ms - drone_swing_pulse_start_ms >= DRONE_STICK_MIN_VISIBLE_PULSE_MS) {
                            drone_swing_display_percent = 0;
                        }

                        if (lift_value != 0) {
                            if (drone_lift_display_percent == 0) {
                                drone_lift_pulse_start_ms = now_ms;
                            }
                            drone_lift_display_percent = lift_value;
                        } else if (drone_lift_display_percent != 0
                                   && now_ms - drone_lift_pulse_start_ms >= DRONE_STICK_MIN_VISIBLE_PULSE_MS) {
                            drone_lift_display_percent = 0;
                        }

                        if (pan_value != 0) {
                            if (drone_pan_display_percent == 0) {
                                drone_pan_pulse_start_ms = now_ms;
                            }
                            drone_pan_display_percent = pan_value;
                        } else if (drone_pan_display_percent != 0
                                   && now_ms - drone_pan_pulse_start_ms >= DRONE_STICK_MIN_VISIBLE_PULSE_MS) {
                            drone_pan_display_percent = 0;
                        }

                        if (tilt_value != 0) {
                            if (drone_tilt_display_percent == 0) {
                                drone_tilt_pulse_start_ms = now_ms;
                            }
                            drone_tilt_display_percent = tilt_value;
                        } else if (drone_tilt_display_percent != 0
                                   && now_ms - drone_tilt_pulse_start_ms >= DRONE_STICK_MIN_VISIBLE_PULSE_MS) {
                            drone_tilt_display_percent = 0;
                        }

                        drone_swing_display_state = (drone_swing_display_percent > 0) ? DRONE_HORIZ_RIGHT
                                                  : (drone_swing_display_percent < 0) ? DRONE_HORIZ_LEFT
                                                  : DRONE_HORIZ_NEUTRAL;
                        drone_lift_display_state = (drone_lift_display_percent > 0) ? DRONE_LIFT_UP
                                                 : (drone_lift_display_percent < 0) ? DRONE_LIFT_DOWN
                                                 : DRONE_LIFT_NEUTRAL;
                        drone_pan_display_state = (drone_pan_display_percent > 0) ? DRONE_HORIZ_RIGHT
                                                : (drone_pan_display_percent < 0) ? DRONE_HORIZ_LEFT
                                                : DRONE_HORIZ_NEUTRAL;
                        drone_tilt_display_state = (drone_tilt_display_percent > 0) ? DRONE_TILT_UP
                                                 : (drone_tilt_display_percent < 0) ? DRONE_TILT_DOWN
                                                 : DRONE_TILT_NEUTRAL;

                        update_drone_lift_indicator();
                        update_drone_tilt_indicator();
                        update_drone_stick_colors();
                        xSemaphoreGive(lvgl_mux);
                    }
                }
            }

            line_len = 0;
            continue;
        }

        if (line_len < sizeof(line) - 1) {
            line[line_len++] = (char)byte;
        } else {
            line_len = 0;
        }
    }
}

#if TEST_FORCE_CONTROLLER_ERROR
static void show_controller_error_cb(lv_timer_t *timer)
{
    (void)timer;
    switch_display_mode(DISPLAY_MODE_ERROR, "NO CONTROLLER\nFOUND", esp_timer_get_time() / 1000ULL);
    lv_timer_del(timer);
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
        PIN_NUM_LCD_PCLK,
        PIN_NUM_LCD_DATA0,
        PIN_NUM_LCD_DATA1,
        PIN_NUM_LCD_DATA2,
        PIN_NUM_LCD_DATA3,
        LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* NVS init */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_err);
    }
    load_settings_from_nvs();

    static lv_disp_drv_t disp_drv;

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS, lvgl_flush_ready_cb, &disp_drv);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    panel_io_global = io_handle;

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 && buf2);

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * LVGL_BUF_HEIGHT);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.rounder_cb = lvgl_rounder_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    /* Touch input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&indev_drv);

    const esp_timer_create_args_t tick_timer_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 0);

    const uart_config_t uart_cfg = {
        .baud_rate = STATUS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(STATUS_UART_PORT, STATUS_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(STATUS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(STATUS_UART_PORT, STATUS_UART_TX_PIN, STATUS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(status_uart_task, "status_uart", 4096, NULL, 3, NULL, 1);

    /* I2C for touch */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOUCH_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    xSemaphoreTake(lvgl_mux, portMAX_DELAY);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    label_moco = lv_label_create(lv_scr_act());
    lv_label_set_text(label_moco, "MOCO");
    lv_obj_set_style_text_font(label_moco, &lv_font_montserrat_150, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_moco, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label_moco, 2, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_moco, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    const int32_t moco_target_y = 110;
    lv_obj_set_pos(label_moco, 16, moco_target_y);
    lv_obj_set_style_text_color(label_moco, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_opa(label_moco, LV_OPA_COVER, LV_PART_MAIN);

    label_jib = lv_label_create(lv_scr_act());
    lv_label_set_text(label_jib, "jib");
    lv_obj_set_style_text_font(label_jib, &lv_font_montserrat_150, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_jib, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label_jib, 2, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_jib, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_update_layout(lv_scr_act());
    const int32_t jib_target_y = 260;
    lv_coord_t jib_width = lv_obj_get_width(label_jib);
    lv_obj_set_pos(label_jib, LCD_H_RES - 16 - jib_width, jib_target_y);
    lv_obj_set_style_text_color(label_jib, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_opa(label_jib, LV_OPA_COVER, LV_PART_MAIN);

    drone_title_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_title_label, "DRONE MODE");
    lv_obj_set_style_text_font(drone_title_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(drone_title_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(drone_title_label, LCD_H_RES - 40);
    lv_obj_set_pos(drone_title_label, 20, 42);

    drone_subtitle_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_subtitle_label, "multi-axis live control");
    lv_obj_set_style_text_color(drone_subtitle_label, lv_color_make(180, 180, 180), LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_subtitle_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(drone_subtitle_label, LCD_H_RES - 40);
    lv_obj_set_pos(drone_subtitle_label, 20, 100);

    drone_precision_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_precision_label, "PRECISION");
    lv_obj_set_style_text_color(drone_precision_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(drone_precision_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(drone_precision_label, -4, LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_precision_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(drone_precision_label, 72, 118);
    lv_obj_update_layout(drone_precision_label);

    drone_precision_state_box = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_precision_state_box);
    lv_obj_set_size(drone_precision_state_box, 30, 30);
    lv_obj_set_pos(drone_precision_state_box,
                   lv_obj_get_x(drone_precision_label) + lv_obj_get_width(drone_precision_label) + 14,
                   126);
    lv_obj_set_style_radius(drone_precision_state_box, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_precision_state_box, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_precision_state_box, lv_color_make(200, 200, 200), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_precision_state_box, LV_OPA_COVER, LV_PART_MAIN);

    drone_boost_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_boost_label, "BOOST");
    lv_obj_set_style_text_color(drone_boost_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(drone_boost_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(drone_boost_label, -4, LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_boost_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(drone_boost_label, 390, 118);
    lv_obj_update_layout(drone_boost_label);

    drone_boost_state_box = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_boost_state_box);
    lv_obj_set_size(drone_boost_state_box, 30, 30);
    lv_obj_set_pos(drone_boost_state_box,
                   lv_obj_get_x(drone_boost_label) + lv_obj_get_width(drone_boost_label) + 14,
                   126);
    lv_obj_set_style_radius(drone_boost_state_box, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_boost_state_box, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_boost_state_box, lv_color_make(200, 200, 200), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_boost_state_box, LV_OPA_COVER, LV_PART_MAIN);

    set_drone_modifier_indicator(false, false);

    drone_left_ring = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_left_ring);
    lv_obj_set_size(drone_left_ring, 170, 170);
    lv_obj_set_pos(drone_left_ring, 70, 190);
    lv_obj_set_style_radius(drone_left_ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_left_ring, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_left_ring, lv_color_make(120, 120, 120), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_left_ring, LV_OPA_TRANSP, LV_PART_MAIN);

    drone_right_ring = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_right_ring);
    lv_obj_set_size(drone_right_ring, 170, 170);
    lv_obj_set_pos(drone_right_ring, 360, 190);
    lv_obj_set_style_radius(drone_right_ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_right_ring, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_right_ring, lv_color_make(120, 120, 120), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_right_ring, LV_OPA_TRANSP, LV_PART_MAIN);

    drone_left_stick = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_left_stick);
    lv_obj_set_size(drone_left_stick, 20, 20);
    lv_obj_set_pos(drone_left_stick, DRONE_LEFT_STICK_BASE_X, DRONE_LEFT_STICK_BASE_Y);
    lv_obj_set_style_radius(drone_left_stick, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(drone_left_stick, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_left_stick, LV_OPA_COVER, LV_PART_MAIN);

    drone_right_stick = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_right_stick);
    lv_obj_set_size(drone_right_stick, 20, 20);
    lv_obj_set_pos(drone_right_stick, DRONE_RIGHT_STICK_BASE_X, DRONE_RIGHT_STICK_BASE_Y);
    lv_obj_set_style_radius(drone_right_stick, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(drone_right_stick, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_right_stick, LV_OPA_COVER, LV_PART_MAIN);

    drone_center_line = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_center_line);
    lv_obj_set_size(drone_center_line, 120, 2);
    lv_obj_set_pos(drone_center_line, 240, 274);
    lv_obj_set_style_bg_opa(drone_center_line, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_side(drone_center_line, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_center_line, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_center_line, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_opa(drone_center_line, LV_OPA_50, LV_PART_MAIN);

    drone_flowlapse_bar = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(drone_flowlapse_bar);
    lv_obj_set_size(drone_flowlapse_bar, FLOWLAPSE_BAR_NORMAL_W, FLOWLAPSE_BAR_NORMAL_H);
    lv_obj_set_pos(drone_flowlapse_bar, FLOWLAPSE_BAR_NORMAL_X, FLOWLAPSE_BAR_NORMAL_Y);
    lv_obj_set_style_radius(drone_flowlapse_bar, 6, LV_PART_MAIN);
    lv_obj_set_style_border_width(drone_flowlapse_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(drone_flowlapse_bar, lv_color_make(120, 120, 120), LV_PART_MAIN);
    lv_obj_set_style_bg_color(drone_flowlapse_bar, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_flowlapse_bar, LV_OPA_40, LV_PART_MAIN);

    drone_flowlapse_fill = lv_obj_create(drone_flowlapse_bar);
    lv_obj_remove_style_all(drone_flowlapse_fill);
    lv_obj_set_size(drone_flowlapse_fill, 0, FLOWLAPSE_BAR_NORMAL_H - 6);
    lv_obj_set_pos(drone_flowlapse_fill, 3, 3);
    lv_obj_set_style_radius(drone_flowlapse_fill, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(drone_flowlapse_fill, lv_color_make(0, 180, 255), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(drone_flowlapse_fill, LV_OPA_COVER, LV_PART_MAIN);

    drone_flowlapse_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_flowlapse_label, flowlapse_status_text);
    lv_obj_set_style_text_color(drone_flowlapse_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_flowlapse_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(drone_flowlapse_label, FLOWLAPSE_BAR_NORMAL_W);
    lv_obj_set_pos(drone_flowlapse_label, FLOWLAPSE_BAR_NORMAL_X, FLOWLAPSE_LABEL_NORMAL_Y);

    drone_flowlapse_waypoint_label = lv_label_create(lv_scr_act());
    lv_label_set_text(drone_flowlapse_waypoint_label, "0/0");
    lv_obj_set_style_text_color(drone_flowlapse_waypoint_label, lv_color_make(255, 50, 50), LV_PART_MAIN);
    lv_obj_set_style_text_font(drone_flowlapse_waypoint_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_align(drone_flowlapse_waypoint_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_size(drone_flowlapse_waypoint_label, 160, 70);
    lv_obj_set_pos(drone_flowlapse_waypoint_label, 250, 368);
    lv_obj_add_flag(drone_flowlapse_waypoint_label, LV_OBJ_FLAG_HIDDEN);

    set_drone_mode_visible(false);

    /* Create settings panels + long-press timer */
    create_settings_list();
    create_editor_panel();
    lv_timer_create(long_press_timer_cb, 50, NULL);

    /* Apply persisted theme */
    apply_theme();

    lv_timer_handler();

    uint8_t brightness_ctrl = 0x20;
    uint8_t initial_brightness = 0;
    ESP_ERROR_CHECK(sh8601_tx_param_qspi(io_handle, 0x53, &brightness_ctrl, 1));
    ESP_ERROR_CHECK(sh8601_tx_param_qspi(io_handle, 0x51, &initial_brightness, 1));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    int target_brightness = (settings[SETTING_BRIGHTNESS].value * 255) / 100;
    lv_anim_t anim_reveal;
    lv_anim_init(&anim_reveal);
    lv_anim_set_var(&anim_reveal, io_handle);
    lv_anim_set_exec_cb(&anim_reveal, anim_set_panel_brightness);
    lv_anim_set_values(&anim_reveal, 0, target_brightness);
    lv_anim_set_time(&anim_reveal, 12000);
    lv_anim_set_path_cb(&anim_reveal, lv_anim_path_ease_in_out);
    lv_anim_start(&anim_reveal);

#if TEST_FORCE_CONTROLLER_ERROR
    lv_timer_create(show_controller_error_cb, TEST_FORCE_CONTROLLER_ERROR_DELAY_MS, NULL);
#endif

    xSemaphoreGive(lvgl_mux);

    ESP_LOGI(TAG, "RIG rendered via LVGL");
}
