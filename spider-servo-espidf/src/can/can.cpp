#include "can.h"

static const char *TAG = "SPIDER_MAIN_CONTROLLER";


void CAN::setup(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps (common CAN speed)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // g_config.tx_queue_len = 1;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;

    // Install & start CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "CAN Driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install CAN driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "CAN started");
    } else {
        ESP_LOGE(TAG, "Failed to start CAN");
        return;
    }
}