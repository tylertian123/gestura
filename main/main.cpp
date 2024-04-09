#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "pose_integrator.hpp"
#include "stream.hpp"
#include "util.hpp"


static const char *TAG = "main";

algo::PoseIntegrator pose_int;
io::DataStreamer datastream;

QueueHandle_t msg_queue;

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    msg_queue = xQueueCreate(5, sizeof(io::Message));

    if (!pose_int.init()) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        abort();
    }
    pose_int.start_task(msg_queue);

    ESP_LOGI(TAG, "Started pose integrator");

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
        DELAY_MS(1000);
    }
}
