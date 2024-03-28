#pragma once

#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"

namespace hw {
    class FlexSensorArray {
    private:
        adc_continuous_handle_t adc_handle = {0};

    public:
        FlexSensorArray() {};

        int16_t short_raw = -1;
        int16_t long_raw = -1;

        esp_err_t init();
        esp_err_t stop();
        
        // Read values from ADC pool and set long_raw and short_raw
        esp_err_t read_raw();
    };
}
