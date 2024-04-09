#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pose_integrator.hpp"
#include "stream.hpp"
#include "util.hpp"


static const char *TAG = "main";

algo::PoseIntegrator pose_int;
io::DataStreamer datastream;

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    // if (!pose_int.init()) {
    //     ESP_LOGE(TAG, "Failed to initialize IMU");
    //     return;
    // }
    // if (!pose_int.calibrate_imu()) {
    //     ESP_LOGE(TAG, "Calibration failed");
    // }
    // else {
    //     ESP_LOGE(TAG, "Calibration successful");
    // }
    // pose_int.start_task();
    
    // ESP_LOGI(TAG, "Started pose integrator");

    ESP_ERROR_CHECK(datastream.init());
    datastream.start_task();

    while (true) {
        // Do nothing here for now
        // FreeRTOS will context switch to the pose integrator task for us
        DELAY_MS(1000);
    }
}
