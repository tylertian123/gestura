#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"

#include "esp_err.h"

#define _CAT(name, n) name ## n
// Concatenate a name and another macro that can expand to a number
// e.g. if DEV_PIN is defined as 1, then CAT(GPIO_NUM_, DEV_PIN) expands to GPIO_NUM_1
// This is used to convert int types in sdkconfig.h to the correct macros
#define CAT(name, n) _CAT(name, n)

#define RETURN_ON_ERR(x) do {                                           \
        esp_err_t err_rc_ = (x);                                        \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            return err_rc_;                                             \
        }                                                               \
    } while(0)

#define DELAY_MS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
