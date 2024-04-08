#include "stream.hpp"

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "nvs_flash.h"

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

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_ap();

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
}
