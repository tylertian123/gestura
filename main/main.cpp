#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio.hpp"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "portmacro.h"
#include "soc/soc_caps.h"

static const char *TAG = "main";

adc_continuous_handle_t handle{0};

void adc_init() {

    adc_continuous_handle_cfg_t handle_cfg = {
        // If this is only set to 1 times the bytes per conv, it will stop working after a few samples
        .max_store_buf_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2,
        .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2,
        .flags = {
            // Unclear what this does
            .flush_pool = false,
        }
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));

    adc_digi_pattern_config_t pattern[1];
    pattern[0] = {
        // Attn of 6 dB means a range of approx. 2.19 V, matching the circuit's max voltage
        .atten = ADC_ATTEN_DB_6,
        .channel = ADC_CHANNEL_5,
        .unit = ADC_UNIT_1,
        // 12 bits
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,

    };
    adc_continuous_config_t ch_cfg = {
        .pattern_num = 1,
        .adc_pattern = pattern,
        // Lowest possible frequency for demo
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        // Type2 seems to be the only one that's valid for ESP32-C6
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    ESP_ERROR_CHECK(adc_continuous_config(handle, &ch_cfg));
    ESP_LOGI(TAG, "Finished ADC setup");
}


extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    hw::GPIO led(4, GPIO_MODE_OUTPUT);

    adc_init();
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    // Make sure we have samples
    vTaskDelay(1);

    while (true) {
        uint8_t buf[4];
        uint32_t result_len = 0;
        esp_err_t err = adc_continuous_read(handle, buf, sizeof(buf), &result_len, 0);
        if (err == ESP_OK) {
            adc_digi_output_data_t *data = reinterpret_cast<adc_digi_output_data_t*>(&buf);
            if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                ESP_LOGW(TAG, "ADC invalid channel num");
                break;
            }
            else {
                led = data->type2.data > 3000;
            }
        }
        else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "ADC timeout");
            break;
        }
        else {
            ESP_ERROR_CHECK(err);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
