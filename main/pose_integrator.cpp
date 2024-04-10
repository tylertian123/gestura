#include "pose_integrator.hpp"

#include "BNO08x.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/projdefs.h"
#include "stream.hpp"

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

    void PoseIntegrator::start_task(QueueHandle_t out_queue) {
        queue = out_queue;
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

            int64_t cur_time = esp_timer_get_time();
            if (cur_time - self->last_send > REPORT_PERIOD) {
                self->last_send = cur_time;
                io::Message msg = {
                    .type = io::Message::Type::POSITION,
                    .position = {
                        .x = self->pos(0),
                        .y = self->pos(1),
                        .z = self->pos(2),
                    }
                };
                if (xQueueSend(self->queue, &msg, 0) != pdTRUE) {
                    //ESP_LOGW(TAG, "Could not send pose: queue full");
                }
            }

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

                    // Variance code
                    self->next_index = (self->index + 1) % WINSIZE;    // Oldest accel value is at next_index, wrapping if needed
                    
                    // Store re-used intermediate variables
                    float old_mean_accel = self->mean_accel;
                    float new_accel = self->accel.norm();
                    float outgoing_accel = self->accels[self->next_index];
                    
                    // Recurvise formula (modification of Welford algo)
                    self->mean_accel += (new_accel - outgoing_accel) / WINSIZE;
                    self->var_sum_accel += (new_accel - old_mean_accel) * (new_accel - self->mean_accel) - (outgoing_accel - old_mean_accel) * (outgoing_accel - self->mean_accel);
                    
                    // Updates for next iter
                    self->accels[self->next_index] = new_accel;
                    self->index = self->next_index;

                    // Curr estimate for var of accel's norm
                    float var_accel = self->var_sum_accel / (WINSIZE-1);

                    // Check if stationary, use as opportunity to correct drift
                    if (var_accel < VAR_THRES) { // Relies on rotation being correct
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
                        ESP_LOGI(TAG, "P: <%f, %f, %f> V: <%f, %f, %f> A: <%f, %f, %f> Var: <%f>", self->pos(0), self->pos(1), self->pos(2), self->vel(0), self->vel(1), self->vel(2), self->accel_local(0), self->accel_local(1), self->accel_local(2), var_accel);
                    }

                    self->start_time = self->end_time;
                }
            }
        }
    }

    void PoseIntegrator::re_zero() {
        self->pos(0) = 0;
        self->pos(1) = 0;
        self->pos(2) = 0;
        self->vel(0) = 0;
        self->vel(1) = 0;
        self->vel(2) = 0;
    }

    bool PoseIntegrator::calibrate_imu() {
        return imu.run_full_calibration_routine();
    }
}
