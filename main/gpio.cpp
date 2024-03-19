#include "gpio.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <stdint.h>

namespace hw {
    GPIO::GPIO(gpio_num_t pin) : num(pin) {}

    GPIO::GPIO(uint8_t pin) : GPIO(static_cast<gpio_num_t>(pin)) {}

    GPIO::GPIO(gpio_num_t pin, gpio_mode_t mode) : GPIO(pin) {
        reinit(mode);
    }

    GPIO::GPIO(uint8_t pin, gpio_mode_t mode) : GPIO(static_cast<gpio_num_t>(pin), mode) {}

    void GPIO::reinit(gpio_mode_t mode) {
        // Never errors
        gpio_reset_pin(num);
        // Errors on invalid GPIO number
        // We can just abort since this won't happen without a code error
        ESP_ERROR_CHECK(gpio_set_direction(num, mode));
    }

    bool GPIO::operator=(bool val) {
        // Errors on invalid GPIO number
        // We can just abort since this won't happen without a code error
        ESP_ERROR_CHECK(gpio_set_level(num, val));
        return val;
    }

    GPIO::operator bool() const {
        return gpio_get_level(num);
    }
}
