#pragma once

#include "esp_err.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

namespace io {
    struct Message {
    public:
        enum Gesture : int8_t {
            INVALID = -1,
            CALIBRATION = 0,
            WRITE = 1,
            ERASE = 2,
        };
        struct Position {
            float x;
            float y;
            float z;
        };

        enum Type : uint8_t {
            GESTURE = 0,
            POSITION = 1,
        } type;
        uint8_t _padding[3] = {0};
        union {
            Gesture gesture;
            Position position;
        };

        static constexpr size_t SIZE_GESTURE = sizeof(type) + sizeof(_padding) + sizeof(gesture);
        static constexpr size_t SIZE_POSITION = sizeof(type) + sizeof(_padding) + sizeof(position);
    };

    class DataStreamer {
    private:
        esp_netif_t *netif_inst = nullptr;

        QueueHandle_t queue;

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
        void start_task(QueueHandle_t in_queue);
    };
}
