#pragma once

#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_err.h"

#include "sdkconfig.h"

#include "util.hpp"
#include "stream.hpp"

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
        int8_t short_bucket = HIGH;
        int8_t long_bucket = HIGH;
        // Hysteresis Threshold
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_HIGH = 4000;
        static constexpr int16_t LONG_THRESHOLD_HIGHRANGE_LOW = 3550; // High threshold for High range
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_HIGH = 3650; // Low threshold for High range 
        static constexpr int16_t LONG_THRESHOLD_MIDRANGE_LOW = 3300;
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_HIGH = 3800; 
        static constexpr int16_t SHORT_THRESHOLD_HIGHRANGE_LOW = 2900;
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_HIGH = 3100; 
        static constexpr int16_t SHORT_THRESHOLD_MIDRANGE_LOW = 2400; 
        // Invalid Value Threshold
        static constexpr int16_t INVALID_THRESHOLD = 2000;
        // Define Gesture Buckets
        static constexpr int8_t LOW = 0; // BENT
        static constexpr int8_t MID = 1;
        static constexpr int8_t HIGH = 2; // FLAT
        // Variables for Gesture Recognition
        static constexpr int8_t ERASE_CONDITION[2] = {LOW, LOW}; // {short, long}
        static constexpr int8_t WRITE_CONDITION[2] = {MID, MID};
        static constexpr int8_t CALIBRATE_CONDITION[2] = {HIGH, HIGH};
        // Gestures
        io::Message::Gesture gesture = io::Message::Gesture::INVALID;
        io::Message::Gesture last_gesture = io::Message::Gesture::INVALID; // Keep different from gesture to begin with

        bool change_status = true;

        esp_err_t init();
        esp_err_t stop();
        
        // Read values from ADC pool and set long_raw and short_raw
        esp_err_t read_raw();
        // Bucket Values into Gesture Ranges
        void bucket_values();
        void get_gesture();
        bool set_gesture_status();
        bool get_gesture_status();
    };
}
