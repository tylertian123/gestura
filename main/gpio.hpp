#pragma once

#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <stdint.h>

namespace hw {
    class GPIO {
    private:
        const gpio_num_t num;
    
    public:
        /// @brief Constructor, leaves the pin uninitialized.
        GPIO(gpio_num_t pin);
        /// @brief Constructor, leaves the pin uninitialized.
        GPIO(uint8_t pin);
        /// @brief Constructor.
        /// @param pin GPIO pin number
        /// @param mode Pin mode
        GPIO(gpio_num_t pin, gpio_mode_t mode);
        /// @brief Constructor.
        /// @param pin GPIO pin number
        /// @param mode Pin mode
        GPIO(uint8_t pin, gpio_mode_t mode);

        /// @brief Reinitialize the GPIO pin in a new direction
        /// @param out Pin mode
        void reinit(gpio_mode_t mode);

        /// @brief Set the output logic level on the pin.
        /// @param val The output to set
        /// @return The output value
        bool operator=(bool val);
        /// @brief Get the reading on the GPIO pin.
        operator bool() const;
    };
}
