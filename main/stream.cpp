#include "stream.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "portmacro.h"

#include "sdkconfig.h"
#include "util.hpp"

namespace io {
    esp_err_t DataStreamer::init() {
        esp_err_t ret = nvs_flash_init();
        RETURN_ON_ERR(ret);
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            RETURN_ON_ERR(nvs_flash_erase());
            RETURN_ON_ERR(nvs_flash_init());
        }

        RETURN_ON_ERR(esp_netif_init());
        RETURN_ON_ERR(esp_event_loop_create_default());
        netif_inst = esp_netif_create_default_wifi_ap();
        
        esp_netif_ip_info_t ip_info;
        ip_info.ip.addr = ESP_IP4TOADDR(192, 168, 0, 1);
        ip_info.gw.addr = ip_info.ip.addr;
        ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
        RETURN_ON_ERR(esp_netif_dhcps_stop(netif_inst));
        RETURN_ON_ERR(esp_netif_set_ip_info(netif_inst, &ip_info));
        RETURN_ON_ERR(esp_netif_dhcps_start(netif_inst));

        wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
        RETURN_ON_ERR(esp_wifi_init(&init_cfg));
        
        wifi_config_t wifi_cfg = {
            .ap = {
                .ssid = CONFIG_STREAM_WIFI_SSID,
                .password = CONFIG_STREAM_WIFI_PASSWD,
                .ssid_len = 0,
                .channel = CONFIG_STREAM_WIFI_CHAN,
                .authmode = WIFI_AUTH_WPA2_PSK,
                .ssid_hidden = 0,
                .max_connection = 1,
            }
        };
        RETURN_ON_ERR(esp_wifi_set_mode(WIFI_MODE_AP));
        RETURN_ON_ERR(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
        RETURN_ON_ERR(esp_wifi_start());

        return ESP_OK;
    }

    void DataStreamer::start_task(QueueHandle_t in_queue) {
        queue = in_queue;
        // Use priority of 1 for this task (larger number = higher priority)
        // Note main task has priority 1, pose integrator priority 4 and IMU SPI task has priority 8
        task_handle = xTaskCreateStatic(main_task_static, "data_streamer", STACK_SIZE, 
            this, 2, task_stack, &task_tcb);
    }

    void DataStreamer::main_task_static(void *params) {
        DataStreamer *self = reinterpret_cast<DataStreamer*>(params);
        self->main_task();
    }

    int DataStreamer::check_fatal(int ret, const char *msg) {
        int e = errno;
        if (likely(ret >= 0)) {
            return ret;
        }
        ESP_LOGE(TAG, "Critical failure (%d): %s: %s", ret, msg, strerror(e));
        // TODO: Better handling here
        // Stop task, clean up socket, etc?
        abort();
    }

    void DataStreamer::main_task() {
        sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        // The htonx macros (where x denotes size) convert from hardware to network byte order
        // Since the system endianness might be different than the network endianness
        dest_addr.sin_port = htons(CONFIG_STREAM_PORT);
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        listen_sock = check_fatal(socket(AF_INET, SOCK_STREAM, IPPROTO_IP), "socket");
        int opt = 1;
        check_fatal(setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)), "setsockopt");
        check_fatal(bind(listen_sock, reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr)), "bind");
        check_fatal(listen(listen_sock, 1), "listen");

        ESP_LOGI(TAG, "Server listening");

        while (true) {
            sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            client_sock = check_fatal(accept(listen_sock, reinterpret_cast<sockaddr*>(&client_addr), &addr_len), "accept");
            ESP_LOGI(TAG, "Accepted client");

            // TODO: setsockopt, TCP_NODELAY?

            // TODO: actually send stuff
            // Reset the queue, so old messages don't get sent
            xQueueReset(queue);
            while (true) {
                Message msg;
                // TODO: Add proper handling here
                xQueueReceive(queue, &msg, portMAX_DELAY);
                uint8_t buf[sizeof(Message)];
                size_t buf_size = 0;

                if (msg.type == Message::Type::GESTURE) {
                    buf_size = Message::SIZE_GESTURE;
                }
                else if (msg.type == Message::Type::POSITION) {
                    buf_size = Message::SIZE_POSITION;
                }
                else {
                    ESP_LOGW(TAG, "Invalid message type received: %d", msg.type);
                }
                
                if (buf_size) {
                    memcpy(buf, &msg, buf_size);
                    ssize_t bytes_left = buf_size;
                    while (bytes_left) {
                        // TODO: Add proper handling for connection close
                        bytes_left -= check_fatal(send(client_sock, buf + (buf_size - bytes_left), bytes_left, 0), "send");
                    }
                    ESP_LOGD(TAG, "Sent message of type %d, len %d", msg.type, buf_size);
                }
            }

            shutdown(client_sock, SHUT_RD);
            close(client_sock);
        }
    }
}
