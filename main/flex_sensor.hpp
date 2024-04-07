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

        esp_err_t init();
        esp_err_t stop();
        
        // Read values from ADC pool and set long_raw and short_raw
        esp_err_t read_raw();
    };
}
