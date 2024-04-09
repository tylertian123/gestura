#pragma once

#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_err.h"

#include "sdkconfig.h"

#include "util.hpp"

#define FLEX_SHORT_CHAN CAT(ADC_CHANNEL_, CONFIG_FLEX_SHORT_ADC_CHAN)
#define FLEX_LONG_CHAN CAT(ADC_CHANNEL_, CONFIG_FLEX_LONG_ADC_CHAN)
#define FLEX_UNIT CAT(ADC_UNIT_, CONFIG_FLEX_ADC_UNIT)
#define FLEX_CONV_MODE CAT(ADC_CONV_SINGLE_UNIT_, CONFIG_FLEX_ADC_UNIT)

namespace hw {
    class FlexSensorArray {
    private:
        adc_continuous_handle_t adc_handle = {0};
        adc_iir_filter_handle_t filt_long = {0};
        adc_iir_filter_handle_t filt_short = {0};

    public:
        FlexSensorArray() {};
        ~FlexSensorArray();

        int16_t short_raw = -1;
        int16_t long_raw = -1;

        // Variables and Constants for Bucketing
        int8_t short_bucket = -1;
        int8_t long_bucket = -1;
        // Hysteresis Threshold
        // - anything below 2000 skip value
        // max reading 4081
        // full bend, a little under 3000
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_HIGH = 3380; // High threshold for Middle range
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_LOW = 3280; // Low threshold for Middle range
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_HIGH = 3710;
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_LOW = 3610;
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_HIGH = 3380;
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_LOW = 3280;
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_HIGH = 3710;
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_LOW = 3610;
        // Invalid Value Threshold
        static constexpr INVALID_THRESHOLD = 2900
        // Define Gesture Buckets
        static constexpr int8_t LOW = 0; // BENT
        static constexpr int8_t MID = 1;
        static constexpr int8_t HIGH = 2; // FLAT
        // Variables for Gesture Recognition
        int8_t[2] ERASE = {LOW, LOW}; // {short, long}
        int8_t[2] PEN_DOWN = {MID, MID}
        int8_t[2] PEN_UP = {MID, HIGH}
        int8_t[2] CALIBRATE_POSITION = {HIGH, HIGH}

        esp_err_t init();
        esp_err_t stop();
        
        // Read values from ADC pool and set long_raw and short_raw
        esp_err_t read_raw();
        // Bucket Values into Gesture Ranges
        esp_err_t bucket_values();
    };
}
