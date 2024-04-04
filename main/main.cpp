#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO08x.hpp"

#include "hal/spi_types.h"
#include "util.hpp"


static const char *TAG = "main";

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Program init");

    // TODO: These should use the CONFIG_IMU_x macros as defined in sdkconfig.h from Kconfig
    bno08x_config_t config{SPI2_HOST, GPIO_NUM_21, GPIO_NUM_20, GPIO_NUM_19, 
        GPIO_NUM_22, GPIO_NUM_15, GPIO_NUM_23, GPIO_NUM_NC, 100000UL, true};
    BNO08x imu{config};


    if(!imu.initialize()) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
    }

    //enable gyro & game rotation vector
    imu.enable_rotation_vector(100000UL); //100,000us == 100ms report interval

    while(1)
    {
        //print absolute heading in degrees and angular velocity in Rad/s
        if(imu.data_available())
        {
            ESP_LOGI("Main", "Euler Angle: x (roll): %.3f y (pitch): %.3f z (yaw): %.3f", imu.get_roll_deg(), imu.get_pitch_deg(), imu.get_yaw_deg());
        }
    }
}
