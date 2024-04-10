#pragma once

#include "esp_err.h"
#include "led_strip.h"
#include "led_strip_types.h"

namespace io {
    class ErrorReporter {
    public:
        enum Status {
            UNINIT = 0,
            NORMAL = 1,
            WARN = 2,
            ERROR = 3,
        };

    private:
        Status status;
        led_strip_handle_t led;

        static constexpr const char *TAG = "err_reporter";

    public:
        ErrorReporter() {}

        esp_err_t init();
        void set_status(Status status);

        void error(const char *component);
        void warn(const char *component);
    };

    extern ErrorReporter err_reporter;
}
