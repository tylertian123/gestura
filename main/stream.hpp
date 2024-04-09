#pragma once

#include "esp_err.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

namespace io {
    class DataStreamer {
    private:
        esp_netif_t *netif_inst = nullptr;

        int listen_sock = -1;
        int client_sock = -1;

        // Increase this if we run out
        // StackType_t is uint8_t so this is in bytes
        static constexpr size_t STACK_SIZE = 4096;
        TaskHandle_t task_handle = nullptr;
        StaticTask_t task_tcb;
        StackType_t task_stack[STACK_SIZE];

        int check_fatal(int ret, const char *msg);

        void main_task();
        static void main_task_static(void *params);

        static constexpr const char *TAG = "data_streamer";
    
    public:
        DataStreamer() {};

        esp_err_t init();
        void start_task();
    };
}
