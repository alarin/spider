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

#define INPUT_BUFFER_SIZE 100

static const char *TAG = "SPIDER_MAIN_CONTROLLER";

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

extern "C" {
    void app_main(void);
}


void processCommand(char* str, char *cmd, uint8_t *params) {
    char *token = strtok(str, " "); // Initialize with the string and delimiter
    uint8_t pos = 0;
    while (token != NULL) {
        if (pos == 0) {
            strcpy(cmd, token);
        } else {
            params[pos] = atof(token);
        }
        pos++;
        token = strtok(NULL, " "); // Continue splitting the remaining string
    }
}

void logStatus() {
    uint32_t alerts_triggered;
    twai_status_info_t status_info;

    
    // Wait indefinitely for alerts
    if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100)) == ESP_OK) {
        // Log triggered alerts
        if (alerts_triggered & TWAI_ALERT_RX_DATA) {
            ESP_LOGI(TAG, "Alert: Received message");
        }
        if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGI(TAG, "Alert: Transmission successful");
        }
        if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
            ESP_LOGE(TAG, "Alert: Transmission failed");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGE(TAG, "Alert: Bus error detected");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "Alert: Bus-off state entered");
            // Initiate bus recovery if needed
            twai_initiate_recovery();
        }
        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "Alert: Error passive state");
        }
        if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
            ESP_LOGW(TAG, "Alert: Arbitration lost");
        }
        if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGE(TAG, "Alert: RX queue full - data loss");
        }
    }

    // Optional: Read and log detailed status info
    if (twai_get_status_info(&status_info) == ESP_OK) {
        ESP_LOGI(TAG, "Status %d: TX err cnt=%ld, RX err cnt=%ld", 
                    status_info.state, status_info.tx_error_counter, status_info.rx_error_counter);
        ESP_LOGI(TAG, "Status %d: TX failed=%ld, RX missed=%ld", 
                    status_info.state, status_info.tx_failed_count, status_info.rx_missed_count);
    }    
}

void sendAngle(uint8_t legn, uint8_t motorn, float angle) {
    twai_message_t tx_msg;

    tx_msg.data_length_code = sizeof(motor_command_t);
    tx_msg.identifier = createMsgId(MOTOR_COMMAND, legn, motorn);
    motor_command_t cmd = {SET_ANGLE, angle};
    memcpy(&tx_msg.data, &cmd, sizeof(motor_command_t));
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.identifier);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.data, tx_msg.data_length_code);
    logStatus();
}

void requestStatus(uint8_t legn, uint8_t motorn) {
    twai_message_t tx_msg;

    tx_msg.data_length_code = sizeof(motor_command_t);
    tx_msg.identifier = createMsgId(MOTOR_COMMAND, legn, motorn);
    motor_command_t cmd = {REQUEST_STATUS, 0};
    memcpy(&tx_msg.data, &cmd, sizeof(motor_command_t));
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.identifier);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.data, tx_msg.data_length_code);
    logStatus();
}

void app_main(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 1;
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI(TAG, "Alerts configured");
    } else {
        ESP_LOGE(TAG, "Failed to configure alerts");
    }    

    uint8_t ch;    
    char inputBuffer[INPUT_BUFFER_SIZE + 1]; 
    char cmd;
    uint8_t cmdParams[5]; 
    uint8_t inputPos = 0;
    

    while (1) {
	    ch = getchar();
	    if (ch != 0xFF) {            
		    if (ch == '\n' || inputPos >= INPUT_BUFFER_SIZE) {
                inputBuffer[inputPos] = '\0';
                ESP_LOGI(TAG, "Command recieved %s", inputBuffer);
                processCommand(inputBuffer, &cmd, cmdParams);

                switch(cmd) {
                    case 'a': 
                        sendAngle(cmdParams[1], cmdParams[2], cmdParams[3]);
                        break;
                    case 's':
                        requestStatus(cmdParams[1], cmdParams[2]);
                        break;
                }
                
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }                    
        // sendAngle(1,1,100);
        // logStatus();
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            ESP_LOGI(TAG, "Received CAN ID: 0x%lx", rx_msg.identifier);
            ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.data_length_code);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

