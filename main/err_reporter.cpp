#include "err_reporter.hpp"

#include <cstdlib>

#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_types.h"

#include "util.hpp"

namespace io {
    ErrorReporter err_reporter;

    esp_err_t ErrorReporter::init() {
        led_strip_config_t strip_conf = {
            .strip_gpio_num = 8,
            .max_leds = 1,
            .led_pixel_format = LED_PIXEL_FORMAT_GRB,
            .led_model = LED_MODEL_WS2812,
            .flags = {
                .invert_out = false
            }
        };
        led_strip_rmt_config_t rmt_conf = {
            .resolution_hz = 10 * 1000 * 1000,
            .mem_block_symbols = 0,
            .flags = {
                .with_dma = false
            }
        };
        RETURN_ON_ERR(led_strip_new_rmt_device(&strip_conf, &rmt_conf, &led));
        set_status(Status::UNINIT);
        return ESP_OK;
    }

    void ErrorReporter::set_status(Status status) {
        this->status = status;
        switch (status) {
        case Status::UNINIT:
            led_strip_set_pixel(led, 0, 0, 0, 255);
            break;
        case Status::WAIT:
            led_strip_set_pixel(led, 0, 0, 255, 255);
            break;
        case Status::NORMAL:
            led_strip_set_pixel(led, 0, 0, 255, 0);
            break;
        case Status::ERROR:
            led_strip_set_pixel(led, 0, 255, 0, 0);
            break;
        case Status::WARN:
            led_strip_set_pixel(led, 0, 255, 255, 0);
            break;
        default:
            break;
        }
        led_strip_refresh(led);
    }

    void ErrorReporter::error() {
        ESP_LOGE(TAG, "Fatal error!");
        set_status(Status::ERROR);
        abort();
    }

    void ErrorReporter::warn() {
        ESP_LOGW(TAG, "Warning!");
        set_status(Status::WARN);
    }
}
