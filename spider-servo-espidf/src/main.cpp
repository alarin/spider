#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "mt6701/mt6701.h"
#include "QuickPID.h"


#define CONFIG_PRINT_DELAY 10
#define INPUT_BUFFER_SIZE 100

static const char *TAG = "spider-servo";

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    /* setup here */
    // initUart();
    uint8_t ch;    
    char inputBuffer[INPUT_BUFFER_SIZE + 1]; 
    uint8_t inputPos = 0;

    while (1) { 
	    ch = getchar();
	    if (ch != 0xFF) {
		    if (ch == '\n' || inputPos >= INPUT_BUFFER_SIZE) {
                inputBuffer[inputPos] = '\0';
                ESP_LOGI(TAG, "command received: %s \n", inputBuffer);
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }        

        vTaskDelay(CONFIG_PRINT_DELAY / portTICK_PERIOD_MS);
    }
}