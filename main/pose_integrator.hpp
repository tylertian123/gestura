#pragma once

#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "portmacro.h"

#include "hal/spi_types.h"
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

#define WINSIZE_NORM 25
#define WINSIZE_FILTER 20

namespace algo {
    class PoseIntegrator {
    private:
        bno08x_config_t imu_cfg{SPI2_HOST, IMU_MOSI, IMU_MISO, IMU_SCLK, 
            IMU_CS, IMU_HINT, IMU_RST, GPIO_NUM_NC, 2000000UL, false};
        BNO08x imu{imu_cfg};

        static constexpr const char *TAG = "poseint";

        // Increase this if we run out
        // StackType_t is uint8_t so this is in bytes
        static constexpr size_t STACK_SIZE = 4096;
        TaskHandle_t task_handle = nullptr;
        StaticTask_t task_tcb;
        StackType_t task_stack[STACK_SIZE];

        QueueHandle_t queue;

        // In IMU frame
        Eigen::Vector3f accel_local{0, 0, 0};

        // In global frame
        Eigen::Vector3f accel{0, 0, 0};
        Eigen::Vector3f last_accel{0, 0, 0};
        Eigen::Vector3f vel{0, 0, 0};
        Eigen::Vector3f last_vel{0, 0, 0};
        Eigen::Vector3f pos{0, 0, 0};

        Eigen::Quaternionf rotation;

        float dt = 0.0;
        int64_t start_time = 0; // microseconds
        int64_t end_time = 0; // microseconds
        int64_t last_send = 0; // microseconds
        static constexpr int64_t REPORT_PERIOD = 20000; // microseconds

        float mean_accel_norm = 0.0;
        float var_sum_accel_norm = 0.0;
        float accels_norm[WINSIZE_NORM] = {0.0};
        uint16_t norm_index = 0;
        uint16_t norm_next_index = 0;
        bool correcting = false;

        Eigen::Vector3f mean_accel{0, 0, 0};
        Eigen::Vector3f last_mean_accel{0, 0, 0};
        Eigen::MatrixXd accels = Eigen::MatrixXd::Constant(3, WINSIZE_FILTER, 0.0);
        uint16_t index = 0;
        uint16_t next_index = 0;

        // Thresholds for determining if stationary (i.e. if we should correct drift)
        static constexpr float VAR_THRES = 0.15f; // Tune (m/s^2)
        static constexpr int64_t TIME_THRES = 100000L; // Tune (us)
        int64_t time_near_zero = 0; // microseconds

        Eigen::Vector3f pos_on_stop{0, 0, 0};

        static void main_task(void *params);

    public:
        PoseIntegrator() {}

        esp_err_t init();
        void start_task(QueueHandle_t out_queue);
        void calibration();
        void erase();

        bool calibrate_imu();
    };
}
