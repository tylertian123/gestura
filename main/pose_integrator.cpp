#include "pose_integrator.hpp"

#include "BNO08x.hpp"
#include "esp_log.h"
#include "esp_timer.h"

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

        self->start_time = self->imu.get_time_stamp();

        uint8_t missing_report_count = 0;

        // TODO: for debug logging only; can be removed later
        uint32_t count = 0;

        while (true) {

            // This will block on a semaphore until data is available
            uint16_t report_id = self->imu.get_readings();
            if (!report_id) {
                missing_report_count ++;
                if (missing_report_count > 5) {
                    ESP_LOGE(TAG, "IMU died!");
                    // TODO: figure out something better
                    abort();
                }
            }
            else {
                missing_report_count = 0;
                if (report_id == BNO08x::SENSOR_REPORTID_LINEAR_ACCELERATION) {
                    // Timing
                    self->end_time = esp_timer_get_time();
                    self->dt = (self->end_time - self->start_time) * 1e-6f; // Convert to sec

                    // Read linear accel values (gravity removed)
                    self->accel_local(0) = self->imu.get_linear_accel_X();
                    self->accel_local(1) = self->imu.get_linear_accel_Y();
                    self->accel_local(2) = self->imu.get_linear_accel_Z();

                    // Get rotation quaternion to global frame
                    self->rotation.x() = self->imu.get_quat_I();
                    self->rotation.y() = self->imu.get_quat_J();
                    self->rotation.z() = self->imu.get_quat_K();
                    self->rotation.w() = self->imu.get_quat_real();

                    // Rotate accel vector
                    //self->rotation.normalize();
                    Eigen::Quaternionf accel_local_q;
                    accel_local_q.vec() = self->accel_local;
                    accel_local_q.w() = 0;
                    Eigen::Quaternionf rotated = self->rotation * accel_local_q * self->rotation.inverse(); // Double check
                    self->accel = rotated.vec();

                    // Integrate to get vel
                    self->vel = self->vel + self->accel * self->dt;

                    // Check if stationary, use as opportunity to correct drift
                    if (self->accel.norm() < ACCEL_THRES) { // Relies on rotation being correct
                        if (self->time_near_zero == 0) { // Beginning of zero accel period
                            self->pos_on_stop = self->pos;
                        }
                        self->time_near_zero += (self->end_time - self->start_time);
                    }
                    else {
                        self->time_near_zero = 0;
                    }

                    // Check if we've had low accel for some time (i.e. a pause)
                    if (self->time_near_zero > TIME_THRES) {
                        // TODO: Correct drift better, maybe don't just blindly reset pose to before the pause
                        self->pos = self->pos_on_stop;

                        self->vel(0) = 0;
                        self->vel(1) = 0;
                        self->vel(2) = 0;
                    }

                    // Integrate to get position
                    self->pos = self->pos + self->vel * self->dt;

                    // TODO: Remove (this is for demo/testing only)
                    count ++;
                    if (count >= 25) {
                        count = 0;
                        // ESP_LOGI(TAG, "dt: %f", self->dt);
                        ESP_LOGI(TAG, "P: <%f, %f, %f> V: <%f, %f, %f> A: <%f, %f, %f>", self->pos(0), self->pos(1), self->pos(2), self->vel(0), self->vel(1), self->vel(2), self->accel_local(0), self->accel_local(1), self->accel_local(2));
                    }

                    self->start_time = self->end_time;
                }
            }
        }
    }

    bool PoseIntegrator::calibrate_imu() {
        return imu.run_full_calibration_routine();
    }
}
