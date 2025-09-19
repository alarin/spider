#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "memory.h"

#include "driver/gpio.h"

#include "twai_proto.h"
#include "twai/twai.h"
#include "joystick/joystick.h"

#define INPUT_BUFFER_SIZE 100

static const char *TAG = "SPIDER_MAIN_CONTROLLER";

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


void app_main(void) {
    Joystick joystick;
    TWAI twai;
    twai.setup();
    joystick.setup();


    uint8_t ch;    
    char inputBuffer[INPUT_BUFFER_SIZE + 1]; 
    char cmd;
    uint8_t cmdParams[5]; 
    uint8_t inputPos = 0;
    uint8_t currentAngle = 200;
    
    //getting initial angle
    motor_status_t motor_status;
    //twai.requestStatus(1, 1);
    //assert(twai.receive(&motor_status, 5000));
    //currentAngle = motor_status.angle;
    ESP_LOGI(TAG, "Initial motor angle %d", currentAngle);
    

    while (1) {    
	    ch = getchar();
	    if (ch != 0xFF) {            
		    if (ch == '\n' || inputPos >= INPUT_BUFFER_SIZE) {
                inputBuffer[inputPos] = '\0';
                ESP_LOGI(TAG, "Command recieved %s", inputBuffer);
                processCommand(inputBuffer, &cmd, cmdParams);

                switch(cmd) {
                    case 'a': 
                        twai.sendAngle(cmdParams[1], cmdParams[2], cmdParams[3]);
                        break;
                    case 's':
                        twai.requestStatus(cmdParams[1], cmdParams[2]);
                        break;
                }
                
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }                    
        // sendAngle(1,1,100);
        // logStatus();
        
        if (joystick.getX() != 0) {
            ESP_LOGI(TAG, "Sending new angle from joystick %d", currentAngle);
            currentAngle += joystick.getX();
            twai.sendAngle(1, 1, currentAngle);
        }
        twai.receive(&motor_status);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

