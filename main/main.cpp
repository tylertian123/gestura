#include "err_reporter.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "err_reporter.hpp"
#include "pose_integrator.hpp"
#include "flex_sensor.hpp"
#include "stream.hpp"
#include "util.hpp"


static const char *TAG = "main";
static constexpr uint64_t REPORT_PERIOD = 200000;

algo::PoseIntegrator pose_int;
hw::FlexSensorArray flex_sensor_array;
io::DataStreamer datastream;

QueueHandle_t msg_queue;

int64_t last_send = 0;

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    ESP_ERROR_CHECK(io::err_reporter.init());

    msg_queue = xQueueCreate(5, sizeof(io::Message));

    CHECK_FATAL_ESP(pose_int.init());
    pose_int.start_task(msg_queue);
    ESP_LOGI(TAG, "Started pose integrator");

    CHECK_FATAL_ESP(flex_sensor_array.init());
    ESP_LOGI(TAG, "Started flex sensors");

    CHECK_FATAL_ESP(datastream.init());
    datastream.start_task(msg_queue);
    ESP_LOGI(TAG, "Started data streamer");

    ESP_LOGI(TAG, "Initialization complete");
    io::err_reporter.set_status(io::ErrorReporter::Status::WAIT);

    while (true) {
        // Do nothing here for now
        // FreeRTOS will context switch to the pose integrator task for us
        ESP_ERROR_CHECK(flex_sensor_array.read_raw());
        ESP_ERROR_CHECK(flex_sensor_array.bucket_values());
        ESP_ERROR_CHECK(flex_sensor_array.get_gesture());

        if (flex_sensor_array.gesture == io::Message::Gesture::CALIBRATION || flex_sensor_array.gesture == io::Message::Gesture::ERASE) {
            pose_int.re_zero();
        }

        int64_t cur_time = esp_timer_get_time();
        if (cur_time - last_send > REPORT_PERIOD) {
            last_send = cur_time;
            io::Message msg = {
                .type = io::Message::Type::GESTURE,
                .gesture = flex_sensor_array.gesture,
            };
            if (xQueueSend(msg_queue, &msg, 0) != pdTRUE) {
                //ESP_LOGW(TAG, "Could not send pose: queue full");
            }
            // ESP_LOGI(TAG, "GESTURE SENT");
            ESP_LOGI(TAG, "GESTURE: %u", flex_sensor_array.gesture);

            flex_sensor_array.change_status = false;
        }
        DELAY_MS(100);
    }
}
