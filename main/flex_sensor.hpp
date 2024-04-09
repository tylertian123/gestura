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
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_HIGH = 3380; // High threshold for Middle range
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_LOW = 3280; // Low threshold for Middle range
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_HIGH = 3710;
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_LOW = 3610;
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_HIGH = 3380;
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_LOW = 3280;
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_HIGH = 3710;
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_LOW = 3610;
        // Invalid Value Threshold
        static constexpr INVALID_THRESHOLD = 2900;
        // Define Gesture Buckets
        static constexpr int8_t LOW = 0; // BENT
        static constexpr int8_t MID = 1;
        static constexpr int8_t HIGH = 2; // FLAT
        // Variables for Gesture Recognition
        static constexpr int8_t ERASE_CONDITION[2] = {LOW, LOW}; // {short, long}
        static constexpr int8_t PEN_DOWN_CONDITION[2] = {MID, MID};
        static constexpr int8_t PEN_UP_CONDITION[2] = {MID, HIGH};
        static constexpr int8_t CALIBRATE_POSITION_CONDITION[2] = {HIGH, HIGH};
        // Gestures
        int8_t gesture = -1;
        static constexpr int8_t CALIBRATE = 0;
        static constexpr int8_t PEN_UP = 1;
        static constexpr int8_t PEN_DOWN = 2;
        static constexpr int8_t ERASE = 3;

        esp_err_t init();
        esp_err_t stop();
        
        // Read values from ADC pool and set long_raw and short_raw
        esp_err_t read_raw();
        // Bucket Values into Gesture Ranges
        esp_err_t bucket_values();
        esp_err_t get_gesture();
    };
}
