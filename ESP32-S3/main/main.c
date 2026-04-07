#include <string.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lvgl.h"
#include "esp_lcd_sh8601.h"

extern const lv_font_t lv_font_montserrat_120;

#define LCD_HOST SPI2_HOST
#define LCD_BIT_PER_PIXEL 16

#define PIN_NUM_LCD_CS (GPIO_NUM_6)
#define PIN_NUM_LCD_PCLK (GPIO_NUM_47)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_18)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_7)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_48)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_5)
#define PIN_NUM_LCD_RST (GPIO_NUM_17)

#define LCD_H_RES 536
#define LCD_V_RES 240

static const char *TAG = "RIG";

#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_STACK_SIZE   (6 * 1024)
#define LVGL_TASK_PRIORITY     2
#define LVGL_BUF_HEIGHT        (LCD_V_RES / 4)

static SemaphoreHandle_t lvgl_mux = NULL;
static esp_lcd_panel_handle_t s_panel_handle = NULL;

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x36, (uint8_t[]){0xF0}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x02, 0x17}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x00, 0xEF}, 4, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
};

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_draw_bitmap(s_panel_handle,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              color_map);
    lv_disp_flush_ready(drv);
}

static void lvgl_rounder_cb(struct _lv_disp_drv_t *drv, lv_area_t *area)
{
    (void)drv;
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

static void lvgl_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "LVGL task started");
    while (1) {
        if (xSemaphoreTake(lvgl_mux, portMAX_DELAY) == pdTRUE) {
            uint32_t delay_ms = lv_timer_handler();
            xSemaphoreGive(lvgl_mux);
            if (delay_ms > 500) delay_ms = 500;
            if (delay_ms < 1)   delay_ms = 1;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing RIG display with LVGL");

    /* ── SPI / panel init ─────────────────────────────────────────── */
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
        PIN_NUM_LCD_PCLK,
        PIN_NUM_LCD_DATA0, PIN_NUM_LCD_DATA1,
        PIN_NUM_LCD_DATA2, PIN_NUM_LCD_DATA3,
        LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8
    );
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config =
        SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS, NULL, NULL);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    sh8601_vendor_config_t vendor_config = {
        .init_cmds      = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags          = { .use_qspi_interface = 1 },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num  = PIN_NUM_LCD_RST,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = LCD_BIT_PER_PIXEL,
        .vendor_config   = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));

    /* ── LVGL init ───────────────────────────────────────────────── */
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(
        LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(
        LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 && buf2);

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * LVGL_BUF_HEIGHT);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res       = LCD_H_RES;
    disp_drv.ver_res       = LCD_V_RES;
    disp_drv.flush_cb      = lvgl_flush_cb;
    disp_drv.rounder_cb    = lvgl_rounder_cb;
    disp_drv.draw_buf      = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* ── LVGL tick timer (2 ms) ──────────────────────────────────── */
    const esp_timer_create_args_t tick_timer_args = {
        .callback = lvgl_tick_cb,
        .name     = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* ── Draw "RIG" ──────────────────────────────────────────────── */
    xSemaphoreTake(lvgl_mux, portMAX_DELAY);

    /* Black background */
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    /* RIG label */
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "BIG RIG");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_120, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label, 8, LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(label);

    xSemaphoreGive(lvgl_mux);

    ESP_LOGI(TAG, "RIG rendered via LVGL");
}
