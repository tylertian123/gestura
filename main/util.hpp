#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"

#include "esp_err.h"

#define _CAT(name, n) name ## n
#define CAT(name, n) _CAT(name, n)

#define RETURN_ON_ERR(x) do {                                           \
        esp_err_t err_rc_ = (x);                                        \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            return err_rc_;                                             \
        }                                                               \
    } while(0)

#define DELAY_MS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
