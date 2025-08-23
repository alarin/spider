#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "memory.h"

#include "twai_proto.h"

static const char *TAG = "SPIDER_MAIN_CONTROLLER";

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_0, GPIO_NUM_1, TWAI_MODE_NORMAL);

extern "C" {
    void app_main(void);
}

void app_main(void) {
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    while(1) {
        twai_message_t tx_msg;
        float angle = 100;
        tx_msg.data_length_code = 5;
        tx_msg.identifier = createMsgId(SET_MOTOR, 1, 1);
        tx_msg.data[0] = 0;
        memcpy(&angle, &tx_msg.data[1], sizeof(angle));
        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
        ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.identifier);
        ESP_LOG_BUFFER_HEX(TAG, tx_msg.data, tx_msg.data_length_code);

        vTaskDelay(pdMS_TO_TICKS(5000)); 
    }
}

