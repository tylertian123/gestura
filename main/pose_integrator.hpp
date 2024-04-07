#pragma once

#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/spi_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#include "BNO08x.hpp"

#include "sdkconfig.h"
#include "util.hpp"

#define IMU_MOSI CAT(GPIO_NUM_, CONFIG_IMU_MOSI)
#define IMU_MISO CAT(GPIO_NUM_, CONFIG_IMU_MISO)
#define IMU_SCLK CAT(GPIO_NUM_, CONFIG_IMU_SCLK)
#define IMU_CS CAT(GPIO_NUM_, CONFIG_IMU_CS)
#define IMU_HINT CAT(GPIO_NUM_, CONFIG_IMU_HINT)
#define IMU_RST CAT(GPIO_NUM_, CONFIG_IMU_RST)

namespace algo {
    class PoseIntegrator {
    private:
        bno08x_config_t imu_cfg{SPI2_HOST, IMU_MOSI, IMU_MISO, IMU_SCLK, 
            IMU_CS, IMU_HINT, IMU_RST, GPIO_NUM_NC, 2000000UL, false};
        BNO08x imu{imu_cfg};

        // Increase this if we run out
        // StackType_t is uint8_t so this is in bytes
        static constexpr size_t STACK_SIZE = 2048;
        TaskHandle_t task_handle = nullptr;
        StaticTask_t task_tcb;
        StackType_t task_stack[STACK_SIZE];

        static void main_task(void *params);

    public:
        PoseIntegrator() {}

        bool init();
        void start_task();
    };
}
