#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

    msg_queue = xQueueCreate(5, sizeof(io::Message));

    if (!pose_int.init()) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        abort();
    }

    ESP_LOGI(TAG, "Started pose integrator");

    pose_int.start_task(msg_queue);

    ESP_ERROR_CHECK(flex_sensor_array.init());

    ESP_LOGI(TAG, "Started flex sensors");

    // if (!pose_int.calibrate_imu()) {
    //     ESP_LOGE(TAG, "Calibration failed");
    // }
    // else {
    //     ESP_LOGE(TAG, "Calibration successful");
    // }
    // pose_int.start_task();

    ESP_ERROR_CHECK(datastream.init());
    datastream.start_task(msg_queue);

    ESP_LOGI(TAG, "Started data streamer");

    while (true) {
        // Do nothing here for now
        // FreeRTOS will context switch to the pose integrator task for us
        ESP_ERROR_CHECK(flex_sensor_array.read_raw());
        ESP_ERROR_CHECK(flex_sensor_array.bucket_values());
        ESP_ERROR_CHECK(flex_sensor_array.get_gesture());

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
            flex_sensor_array.change_status = false;
        }
        DELAY_MS(100);
    }
}
