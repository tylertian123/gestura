#pragma once

#include "esp_err.h"
#include "led_strip.h"
#include "led_strip_types.h"

#define CHECK_FATAL_ESP(x) do {                                         \
        esp_err_t err_rc_ = (x);                                        \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            ESP_ERROR_CHECK_WITHOUT_ABORT(x);                           \
            io::err_reporter.error();                                   \
        }                                                               \
    } while(0)

namespace io {
    class ErrorReporter {
    public:
        enum Status {
            UNINIT = 0,
            WAIT = 1,
            NORMAL = 2,
            WARN = 3,
            ERROR = 4,
        };

    private:
        Status status;
        led_strip_handle_t led;

        static constexpr const char *TAG = "err_reporter";

    public:
        ErrorReporter() {}

        esp_err_t init();
        void set_status(Status status);

        [[noreturn]] void error();
        void warn();
    };

    extern ErrorReporter err_reporter;
}
