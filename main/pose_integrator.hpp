#pragma once

#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

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

        static constexpr const char *TAG = "poseint";

        // Increase this if we run out
        // StackType_t is uint8_t so this is in bytes
        static constexpr size_t STACK_SIZE = 2048;
        TaskHandle_t task_handle = nullptr;
        StaticTask_t task_tcb;
        StackType_t task_stack[STACK_SIZE];

        // In IMU frame
        Eigen::Vector3f accel_local{0, 0, 0};

        // In global frame
        Eigen::Vector3f accel{0, 0, 0};
        Eigen::Vector3f vel{0, 0, 0};
        Eigen::Vector3f pos{0, 0, 0};

        Eigen::Quaternionf rotation;

        float dt = 0.0;
        int64_t start_time = 0; // microseconds
        int64_t end_time = 0; // microseconds

        // Thresholds for determining if stationary (i.e. if we should correct drift)
        static constexpr float ACCEL_THRES = 0.1f; // Tune (m/s^2)
        static constexpr int64_t TIME_THRES = 250000L; // Tune (us)
        int64_t time_near_zero = 0; // microseconds

        Eigen::Vector3f pos_on_stop{0, 0, 0};

        static void main_task(void *params);

    public:
        PoseIntegrator() {}

        bool init();
        void start_task();
    };
}
