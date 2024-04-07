#include "pose_integrator.hpp"
#include "esp_log.h"

namespace algo {
    bool PoseIntegrator::init() {
        if (!imu.initialize()) {
            return false;
        }
        // Enable data types here
        // See datasheet, section 6.9 (nice) https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf
        // Max rates for rotation vector and accelerometer are 400 Hz, but this may not be possible for both simultaneously
        // 5000 us between samples is 200 Hz
        imu.enable_rotation_vector(5000);
        imu.enable_linear_accelerometer(5000);
        return true;
    }

    void PoseIntegrator::start_task() {
        // Use priority of 4 for this task (larger number = higher priority)
        // Note main task has priority 1, and IMU SPI task has priority 8
        task_handle = xTaskCreateStatic(main_task, "pose_integrator", STACK_SIZE, 
            this, 4, task_stack, &task_tcb);
    }

    void PoseIntegrator::main_task(void *params) {
        PoseIntegrator *self = reinterpret_cast<PoseIntegrator*>(params);

        // TODO: Remove
        uint32_t count = 0;
        while (true) {
            // This will block on a semaphore until data is available
            if (self->imu.data_available()) {
                // Do stuff here!

                // TODO: Remove (this is for demo/testing only)
                count ++;
                if (count >= 200) {
                    count = 0;
                    // This will get logged twice a second
                    // Both rot vec and accel are at 200 Hz so in total data is available at 400 Hz
                    // data_available() doesn't return the type of update, but we can modify it to do so easily enough
                    ESP_LOGI("poseint", "Got 200 samples");
                }
            }
        }
    }
}
