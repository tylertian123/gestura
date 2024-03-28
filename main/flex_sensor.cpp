#include "flex_sensor.hpp"
#include "soc/soc_caps.h"
#include "util.hpp"

#include <assert.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "sdkconfig.h"


namespace hw {
    esp_err_t FlexSensorArray::init() {
        adc_continuous_handle_cfg_t handle_cfg = {
            // Total pool only stores the latest frame
            .max_store_buf_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2,
            // Each frame contains 2 conversion results, one for each channel
            .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2,
            .flags = {
                .flush_pool = true,
            }
        };
        RETURN_ON_ERR(adc_continuous_new_handle(&handle_cfg, &adc_handle));

        adc_digi_pattern_config_t pattern[2];
        pattern[0] = {
            // Attn of 6 dB means a range of approx. 2.19 V, matching the circuit's max voltage
            .atten = ADC_ATTEN_DB_6,
            .channel = CAT(ADC_CHANNEL_, CONFIG_FLEX_SHORT_ADC_CHAN),
            .unit = CAT(ADC_UNIT_, CONFIG_FLEX_ADC_UNIT),
            // 12 bits
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,

        };
        pattern[1] = {
            .atten = ADC_ATTEN_DB_6,
            .channel = CAT(ADC_CHANNEL_, CONFIG_FLEX_LONG_ADC_CHAN),
            .unit = CAT(ADC_UNIT_, CONFIG_FLEX_ADC_UNIT),
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        adc_continuous_config_t ch_cfg = {
            .pattern_num = 2,
            .adc_pattern = pattern,
            // Lowest possible frequency for demo
            .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
            .conv_mode = CAT(ADC_CONV_SINGLE_UNIT_, CONFIG_FLEX_ADC_UNIT),
            // Type2 seems to be the only one that's valid for ESP32-C6
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        };

        RETURN_ON_ERR(adc_continuous_config(adc_handle, &ch_cfg));
        RETURN_ON_ERR(adc_continuous_start(adc_handle));
        return ESP_OK;
    }

    esp_err_t FlexSensorArray::stop() {
        RETURN_ON_ERR(adc_continuous_stop(adc_handle));
        RETURN_ON_ERR(adc_continuous_deinit(adc_handle));
        return ESP_OK;
    }

    esp_err_t FlexSensorArray::read_raw() {
        uint8_t buf[SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2];
        uint32_t result_len = 0;
        RETURN_ON_ERR(adc_continuous_read(adc_handle, buf, sizeof(buf), &result_len, 0));
        // Should always read 2 bytes at a time, since that is the frame size
        assert(result_len == SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2);
        
        adc_digi_output_data_t *convs = reinterpret_cast<adc_digi_output_data_t*>(&buf);
        // One of these should always be the short channel and the other the long channel
        if (convs[0].type2.channel < SOC_ADC_CHANNEL_NUM(CAT(ADC_UNIT_, CONFIG_FLEX_ADC_UNIT))) {
            (convs[0].type2.channel == CAT(ADC_CHANNEL_, CONFIG_FLEX_LONG_ADC_CHAN) ?
                long_raw : short_raw) = convs[0].type2.data;
        }
        if (convs[1].type2.channel < SOC_ADC_CHANNEL_NUM(CAT(ADC_UNIT_, CONFIG_FLEX_ADC_UNIT))) {
            (convs[1].type2.channel == CAT(ADC_CHANNEL_, CONFIG_FLEX_LONG_ADC_CHAN) ?
                long_raw : short_raw) = convs[1].type2.data;
        }
        return ESP_OK;
    }
}
