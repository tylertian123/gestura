#include "flex_sensor.hpp"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"
#include "util.hpp"

#include <assert.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_err.h"

namespace hw {
    FlexSensorArray::~FlexSensorArray() {
        ESP_ERROR_CHECK(stop());
    }

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
            .channel = FLEX_SHORT_CHAN,
            .unit = FLEX_UNIT,
            // 12 bits
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,

        };
        pattern[1] = {
            .atten = ADC_ATTEN_DB_6,
            .channel = FLEX_LONG_CHAN,
            .unit = FLEX_UNIT,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        adc_continuous_config_t ch_cfg = {
            .pattern_num = 2,
            .adc_pattern = pattern,
            // Lowest possible frequency is enough
            .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
            .conv_mode = FLEX_CONV_MODE,
            // Type2 seems to be the only one that's valid for ESP32-C6
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        };

        RETURN_ON_ERR(adc_continuous_config(adc_handle, &ch_cfg));

        adc_continuous_iir_filter_config_t filt_cfg = {
            .unit = FLEX_UNIT,
            .channel = FLEX_SHORT_CHAN,
            .coeff = ADC_DIGI_IIR_FILTER_COEFF_16,
        };
        RETURN_ON_ERR(adc_new_continuous_iir_filter(adc_handle, &filt_cfg, &filt_short));
        filt_cfg.channel = FLEX_LONG_CHAN;
        RETURN_ON_ERR(adc_new_continuous_iir_filter(adc_handle, &filt_cfg, &filt_long));

        RETURN_ON_ERR(adc_continuous_start(adc_handle));
        return ESP_OK;
    }

    esp_err_t FlexSensorArray::stop() {
        RETURN_ON_ERR(adc_continuous_stop(adc_handle));
        RETURN_ON_ERR(adc_del_continuous_iir_filter(filt_short));
        RETURN_ON_ERR(adc_del_continuous_iir_filter(filt_long));
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
        if (convs[0].type2.channel < SOC_ADC_CHANNEL_NUM(FLEX_UNIT)) {
            (convs[0].type2.channel == FLEX_LONG_CHAN ? long_raw : short_raw) = convs[0].type2.data;
        }
        if (convs[1].type2.channel < SOC_ADC_CHANNEL_NUM(FLEX_UNIT)) {
            (convs[1].type2.channel == FLEX_LONG_CHAN ? long_raw : short_raw) = convs[1].type2.data;
        }
        return ESP_OK;
    }

    esp_err_t FlexSensorArray::bucket_values() {
        // Hysteresis for short flex sensor
        if (short_raw < INVALID_THRESHOLD) {
            void(0); // No operation in Invalid (keeps previous value)
        } 
        else if (short_bucket == LOW) {
            if (short_raw > SHORT_THRESHOLD_MIDRANGE_HIGH) short_bucket = MID;
        }
        else if (short_bucket == MID) {
            if (short_raw > SHORT_THRESHOLD_MIDRANGE_HIGH) short_bucket = HIGH;
            if (short_raw < SHORT_THRESHOLD_MIDRANGE_LOW) short_bucket = LOW;
        }
        else if (short_bucket == HIGH) {
            if (short_raw < SHORT_THRESHOLD_HIGHRANGE_LOW) short_bucket = MID;
        }

        // Hysteresis for long flex sensor
        if (long_raw < INVALID_THRESHOLD) {
            void(0);
        } 
        else if (long_bucket == LOW) {
            if (long_raw > LONG_THRESHOLD_MIDRANGE_HIGH) long_bucket = MID;
        }
        else if (long_bucket == MID) {
            if (long_raw > LONG_THRESHOLD_MIDRANGE_HIGH) long_bucket = HIGH;
            if (long_raw < LONG_THRESHOLD_MIDRANGE_LOW) long_bucket = LOW;
        }
        else if (long_bucket == HIGH) {
            if (long_raw < LONG_THRESHOLD_HIGHRANGE_LOW) long_bucket = MID;
        }

        return ESP_OK;
    }

    esp_err_t FlexSensorArray::get_gesture(){
        if (short_bucket == CALIBRATE_CONDITION[0] && long_bucket == CALIBRATE_CONDITION[1]) gesture = CALIBRATE;
        if (short_bucket == PEN_UP_CONDITION[0] && long_bucket == PEN_UP_CONDITION[1]) gesture = PEN_UP;
        if (short_bucket == PEN_DOWN_CONDITION[0] && long_bucket == PEN_DOWN_CONDITION[1]) gesture = PEN_DOWN;
        if (short_bucket == ERASE_CONDITION[0] && long_bucket == ERASE_CONDITION[1]) gesture = ERASE;
        return ESP_OK;
    }


}
