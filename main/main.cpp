#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio.hpp"
#include "hal/gpio_types.h"


extern "C" void app_main(void) {
    hw::GPIO led{4, GPIO_MODE_OUTPUT};

    printf("Program init\n");

    bool state = 0;
    while (true) {
        led = state;
        state = !state;
        printf("State changed to %d\n", state);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
