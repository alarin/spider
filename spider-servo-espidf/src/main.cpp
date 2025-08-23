#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "QuickPID.h"
#include "motordriver/motordriver.h"
#include "twai/twai.h"


#include "config.h"

#define CONFIG_PRINT_DELAY 100
#define INPUT_BUFFER_SIZE 100

static const char *TAG = "spider-servo";

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    uint8_t ch;    
    char inputBuffer[INPUT_BUFFER_SIZE + 1]; 
    double commandValue;
    uint8_t inputPos = 0;
    MotorDriver motorDriver;

    MOTOR_TWAI twai;
    motor_commant_t motor_cmd;

    motorDriver.setup();    
    twai.setup(LEG_ID, MOTOR_ID);

    while (1) { 
        if (twai.receive(&motor_cmd)) {
            motorDriver.setTargetAngle(motor_cmd.param);
        }

	    ch = getchar();
	    if (ch != 0xFF) {
		    if (ch == '\n' || inputPos >= INPUT_BUFFER_SIZE) {
                inputBuffer[inputPos] = '\0';
                commandValue = atof(inputBuffer + 2);
                ESP_LOGI(TAG, "command received: %c %.2f \n", inputBuffer[0], commandValue);
                switch(inputBuffer[0]) {
                    case 'a': 
                        motorDriver.setTargetAngle(commandValue);
                        break;
                    case 'p':
                        motorDriver.setP(commandValue);
                        break;
                    case 'i':
                        motorDriver.setI(commandValue);
                        break;
                    case 'd':
                        motorDriver.setD(commandValue);
                        break;
                    case 'z':                        
                        ESP_LOGE(TAG, "Starting PID tuning");
                        motorDriver.startTuning();
                        break;
                    case 'w':                        
                        ESP_LOGE(TAG, "Calibrate current response %.3f", motorDriver.calibrateCurrent(commandValue/100));
                        break;
                }
                
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }                    
        
        //motorDriver.logInfo();
        vTaskDelay(CONFIG_PRINT_DELAY / portTICK_PERIOD_MS);
    }
}