#include "pose_integrator.hpp"

#include "BNO08x.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/projdefs.h"
#include "stream.hpp"

namespace algo {
    esp_err_t PoseIntegrator::init() {
        if (!imu.initialize()) {
            return ESP_FAIL;
        }
        // Enable data types here
        // See datasheet, section 6.9 (nice) https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf
        // Max rates for rotation vector and accelerometer are 400 Hz, but this may not be possible for both simultaneously
        // 5000 us between samples is 200 Hz
        imu.enable_rotation_vector(5000);
        imu.enable_linear_accelerometer(5000);
        return ESP_OK;
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
                        .correcting = self->correcting,
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

                    // Variance code
                    self->norm_next_index = (self->norm_index + 1) % WINSIZE_NORM;    // Oldest accel value is at norm_next_index, wrapping if needed
                    
                    // Store re-used intermediate variables
                    float old_mean_accel_norm = self->mean_accel_norm;
                    float new_accel_norm = self->accel.norm();
                    float outgoing_accel_norm = self->accels_norm[self->norm_next_index];
                    
                    // Recursive formula (modification of Welford algo)
                    self->mean_accel_norm += (new_accel_norm - outgoing_accel_norm) / WINSIZE_NORM;
                    self->var_sum_accel_norm += (new_accel_norm - old_mean_accel_norm) * (new_accel_norm - self->mean_accel_norm) - (outgoing_accel_norm - old_mean_accel_norm) * (outgoing_accel_norm - self->mean_accel_norm);
                    
                    // Updates for next iter
                    self->accels_norm[self->norm_next_index] = new_accel_norm;
                    self->norm_index = self->norm_next_index;

                    // Curr estimate for var of accel's norm
                    float var_accel_norm = self->var_sum_accel_norm / (WINSIZE_NORM-1);

                    // Filtering code
                    self->next_index = (self->index + 1) % WINSIZE_FILTER;

                    Eigen::Vector3f outgoing_accel{0, 0, 0};
                    outgoing_accel(0) = self->accels(0, self->next_index); // TODO: do better
                    outgoing_accel(1) = self->accels(1, self->next_index);
                    outgoing_accel(2) = self->accels(2, self->next_index);

                    self->mean_accel += (self->accel - outgoing_accel) / WINSIZE_FILTER;

                    self->accels(0, self->next_index) = self->accel(0); // TODO: do better
                    self->accels(1, self->next_index) = self->accel(1);
                    self->accels(2, self->next_index) = self->accel(2);
                    self->index = self->next_index;

                    // Integrate to get vel
                    self->vel = self->vel + ((self->last_mean_accel + self->mean_accel) / 2) * self->dt;

                    // Check if stationary, use as opportunity to correct drift
                    if (var_accel_norm < VAR_THRES) { // Relies on rotation being correct
                        if (self->time_near_zero == 0) { // Beginning of zero accel period
                            self->pos_on_stop = self->pos;
                        }
                        self->correcting = true;
                        self->time_near_zero += (self->end_time - self->start_time);
                    }
                    else {
                        self->time_near_zero = 0;
                        self->correcting = false;
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
                    self->pos = self->pos + ((self->last_vel + self->vel) / 2) * self->dt;
                    // self->pos = self->pos + self->last_vel * self->dt + self->mean_accel * self->dt * self->dt / 2;

                    // TODO: Remove (this is for demo/testing only)
                    count ++;
                    if (count >= 25) {
                        count = 0;
                        // ESP_LOGI(TAG, "dt: %f", self->dt);
                        ESP_LOGI(TAG, "P: <%f, %f, %f> V: <%f, %f, %f> A: <%f, %f, %f> Var: <%f>", self->pos(0), self->pos(1), self->pos(2), self->vel(0), self->vel(1), self->vel(2), self->accel_local(0), self->accel_local(1), self->accel_local(2), var_accel_norm);
                    }

                    // Keep track of prev values
                    self->start_time = self->end_time;
                    self->last_vel = self->vel;
                    self->last_accel = self->accel;
                    self->last_mean_accel = self->mean_accel;
                }
            }
        }
    }

    void PoseIntegrator::erase() {
        pos << 0.0f, 0.0f, 0.0f;
        vel << 0.0f, 0.0f, 0.0f;
        accel << 0.0f, 0.0f, 0.0f;
        last_vel << 0.0f, 0.0f, 0.0f;
        last_accel << 0.0f, 0.0f, 0.0f;

        mean_accel_norm = 0.0;
        var_sum_accel_norm = 0.0;
        memset(accels_norm, 0.0,  WINSIZE_NORM * sizeof(accels_norm[0]));
        norm_index = 0;
        norm_next_index = 0;

        mean_accel << 0.0f, 0.0f, 0.0f;
        last_mean_accel << 0.0f, 0.0f, 0.0f;
        accels = Eigen::MatrixXd::Constant(3, WINSIZE_FILTER, 0.0);
        index = 0;
        next_index = 0;

        time_near_zero = 0;

        pos_on_stop << 0.0f, 0.0f, 0.0f;
    }

    void PoseIntegrator::calibration() {
        pos << 0.0f, 0.0f, 0.0f;
        vel << 0.0f, 0.0f, 0.0f;
        accel << 0.0f, 0.0f, 0.0f;
        last_vel << 0.0f, 0.0f, 0.0f;
        last_accel << 0.0f, 0.0f, 0.0f;

        mean_accel_norm = 0.0;
        var_sum_accel_norm = 0.0;
        memset(accels_norm, 0.0,  WINSIZE_NORM * sizeof(accels_norm[0]));
        norm_index = 0;
        norm_next_index = 0;

        mean_accel << 0.0f, 0.0f, 0.0f;
        last_mean_accel << 0.0f, 0.0f, 0.0f;
        accels = Eigen::MatrixXd::Constant(3, WINSIZE_FILTER, 0.0);
        index = 0;
        next_index = 0;

        time_near_zero = 0;

        pos_on_stop << 0.0f, 0.0f, 0.0f;
    }

    bool PoseIntegrator::calibrate_imu() {
        return imu.run_full_calibration_routine();
    }
}
