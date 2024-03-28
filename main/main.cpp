#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "util.hpp"
#include "gpio.hpp"
#include "flex_sensor.hpp"

#include "hal/gpio_types.h"
#include "esp_log.h"

static const char *TAG = "main";

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    hw::GPIO led(4, GPIO_MODE_OUTPUT);

    hw::FlexSensorArray flex;

    ESP_ERROR_CHECK(flex.init());
    // Make sure we have samples
    DELAY_MS(10);

    while (true) {
        esp_err_t err = flex.read_raw();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Long: %" PRId16 "; short: %" PRId16, flex.long_raw, flex.short_raw);
            led = flex.long_raw > flex.short_raw;
        }
        else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "ADC read timeout!");
        }
        else {
            ESP_ERROR_CHECK(err);
        }
        DELAY_MS(100);
    }

    ESP_ERROR_CHECK(flex.stop());
}
