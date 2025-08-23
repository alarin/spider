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
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_0, GPIO_NUM_1, TWAI_MODE_NORMAL);

extern "C" {
    void app_main(void);
}

void sendAngle(uint8_t legn, uint8_t motorn, float angle) {
    twai_message_t tx_msg;

    tx_msg.data_length_code = 5;
    tx_msg.identifier = createMsgId(SET_MOTOR, legn, motorn);
    tx_msg.data[0] = 0;
    memcpy(&tx_msg.data[1], &angle, sizeof(angle));
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.identifier);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.data, tx_msg.data_length_code);
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


void app_main(void) {
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

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
                }
                
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }                    

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

