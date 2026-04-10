#include <string.h>
#include <stdbool.h>

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

extern const lv_font_t lv_font_montserrat_150;
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
#define LVGL_TASK_STACK_SIZE (6 * 1024)
#define LVGL_TASK_PRIORITY 2
#define LVGL_BUF_HEIGHT (LCD_V_RES / 10)
#define TEST_FORCE_CONTROLLER_ERROR 0
#define TEST_FORCE_CONTROLLER_ERROR_DELAY_MS 3000

#define STATUS_UART_PORT UART_NUM_1
#define STATUS_UART_RX_PIN GPIO_NUM_40
#define STATUS_UART_TX_PIN GPIO_NUM_41
#define STATUS_UART_BAUDRATE 9600
#define STATUS_UART_BUF_SIZE 512
#define STATUS_SIGNAL_TIMEOUT_MS 3500

static SemaphoreHandle_t lvgl_mux = NULL;
static lv_obj_t *label_moco = NULL;
static lv_obj_t *label_jib = NULL;
static lv_obj_t *status_label = NULL;
static bool status_error_active = false;

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
            if (delay_ms > 500) {
                delay_ms = 500;
            } else if (delay_ms < 1) {
                delay_ms = 1;
            }
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
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
    if (label_moco) {
        lv_obj_clear_flag(label_moco, LV_OBJ_FLAG_HIDDEN);
    }
    if (label_jib) {
        lv_obj_clear_flag(label_jib, LV_OBJ_FLAG_HIDDEN);
    }
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
                show_status_on_display(NULL, false);
                xSemaphoreGive(lvgl_mux);
            }
            last_status_rx_ms = now_ms;
        }

        int len = uart_read_bytes(STATUS_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
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
            ESP_LOGI(TAG, "Mega status: %s", line);
            last_status_rx_ms = now_ms;

            if (strncmp(line, "CONTROLLER_ERROR:", 17) == 0) {
                const char *msg = line + 17;
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    show_status_on_display(msg, true);
                    xSemaphoreGive(lvgl_mux);
                }
            } else if (strncmp(line, "CONTROLLER_OK:", 14) == 0) {
                if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(250)) == pdTRUE) {
                    show_status_on_display(NULL, false);
                    xSemaphoreGive(lvgl_mux);
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
    show_status_on_display("NO CONTROLLER\nFOUND", true);
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

    static lv_disp_drv_t disp_drv;

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS, lvgl_flush_ready_cb, &disp_drv);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

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

    const esp_timer_create_args_t tick_timer_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

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
    xTaskCreate(status_uart_task, "status_uart", 4096, NULL, 3, NULL);

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

    lv_timer_handler();

    uint8_t brightness_ctrl = 0x20;
    uint8_t initial_brightness = 0;
    ESP_ERROR_CHECK(sh8601_tx_param_qspi(io_handle, 0x53, &brightness_ctrl, 1));
    ESP_ERROR_CHECK(sh8601_tx_param_qspi(io_handle, 0x51, &initial_brightness, 1));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    lv_anim_t anim_reveal;
    lv_anim_init(&anim_reveal);
    lv_anim_set_var(&anim_reveal, io_handle);
    lv_anim_set_exec_cb(&anim_reveal, anim_set_panel_brightness);
    lv_anim_set_values(&anim_reveal, 0, 255);
    lv_anim_set_time(&anim_reveal, 12000);
    lv_anim_set_path_cb(&anim_reveal, lv_anim_path_ease_in_out);
    lv_anim_start(&anim_reveal);

#if TEST_FORCE_CONTROLLER_ERROR
    lv_timer_create(show_controller_error_cb, TEST_FORCE_CONTROLLER_ERROR_DELAY_MS, NULL);
#endif

    xSemaphoreGive(lvgl_mux);

    ESP_LOGI(TAG, "RIG rendered via LVGL");
}
