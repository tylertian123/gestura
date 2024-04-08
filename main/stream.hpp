#pragma once

#include "esp_err.h"

namespace io {
    class DataStreamer {
    private:
    
    public:
        DataStreamer() {};

        esp_err_t init();
    };
}
