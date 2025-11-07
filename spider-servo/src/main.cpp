#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "motordriver/motordriver.h"
#include "motortwai.h"


#include "config.h"
#include "twai_proto.h"

#define CONFIG_PRINT_DELAY 1000
#define INPUT_BUFFER_SIZE 100

static const char *TAG = "spider-servo";

extern "C" {
    void app_main(void);
}


class MotorController {
    private:
        static constexpr uint32_t HEARTBEAT_DELAY_TICKS = pdMS_TO_TICKS(300);

        MotorTWAI twai;
        MT6701 *encoder;
        MotorDriver motorDriver;
        
        void twaiRxTask() {
            motor_command_t motor_cmd;

            for(;;) {
                twai.rxFromQueue(&motor_cmd);
                ESP_LOGI(TAG, "Motor command received");

                if (motor_cmd.command == SET_ANGLE) {
                    motorDriver.setTargetAngle(motor_cmd.param);
                } else if (motor_cmd.command == REQUEST_STATUS) {
                    twai.sendStatus(motor_status_t {
                        (MotorState) motorDriver.getState(), 
                        (float) motorDriver.getCurrentAngle(),
                        static_cast<uint8_t>(motorDriver.getCurrent() * 10)
                    });
                }
            }
        }

        void twaiHeartbeatTask() {
            for(;;) {
                twai.sendStatus(motor_status_t {
                    (MotorState) motorDriver.getState(), 
                    (float) motorDriver.getCurrentAngle(),
                    static_cast<uint8_t>(motorDriver.getCurrent() * 10)
                });
                vTaskDelay(HEARTBEAT_DELAY_TICKS);
            }
        }

        MotorController(MT6701 *encoder) : encoder(encoder), motorDriver(*encoder) {}
    public:
        static MotorController create(bool encoderEmulation) {
            if (encoderEmulation) {
                MT6701Emulator *encoder = new MT6701Emulator(0.5, 1);
                return MotorController(encoder);
            } else {
                MT6701 *encoder = new MT6701();
                return MotorController(encoder);
            }
        }
        
        ~ MotorController() {
            delete encoder;
        }

        void logInfo() {
            motorDriver.logInfo();
        }

        void start() {
            motorDriver.setup(MOTOR_MIN_ANGLE, MOTOR_MAX_ANGLE);    

            twai.setup(LEG_ID, MOTOR_ID, TWAI_TX_PIN, TWAI_RX_PIN);

            xTaskCreatePinnedToCore(
                [](void* arg){ static_cast<MotorController*>(arg)->twaiRxTask(); },
                "twaiRX",
                10000,
                this,
                10,
                NULL,
                1
            );

            xTaskCreatePinnedToCore(
                [](void* arg){ static_cast<MotorController*>(arg)->twaiHeartbeatTask(); },
                "twaiHearbeat",
                10000,
                this,
                10,
                NULL,
                1
            );

        }
};

void app_main(void)
{
    MotorController mc = MotorController::create(true);
    mc.start();

    uint8_t ch;    
    char inputBuffer[INPUT_BUFFER_SIZE + 1]; 
    double commandValue;
    uint8_t inputPos = 0;

    while (1) { 
        // if (twai.receive(&motor_cmd)) {
        //     if (motor_cmd.command == SET_ANGLE) {
        //         motorDriver.setTargetAngle(motor_cmd.param);
        //     } else if (motor_cmd.command == REQUEST_STATUS) {
        //         twai.sendStatus(motor_status_t {(MotorState) motorDriver.getState(), (float) motorDriver.getCurrentAngle()});
        //     }
        // }

	    ch = getchar();
	    if (ch != 0xFF) {
		    if (ch == '\n' || inputPos >= INPUT_BUFFER_SIZE) {
                inputBuffer[inputPos] = '\0';
                commandValue = atof(inputBuffer + 2);
                ESP_LOGI(TAG, "command received: %c %.2f \n", inputBuffer[0], commandValue);
                switch(inputBuffer[0]) {
                    case 'a': 
                        // motorDriver.setTargetAngle(commandValue);
                        break;
                    case 'p':
                        // motorDriver.setP(commandValue);
                        break;
                    case 'i':
                        // motorDriver.setI(commandValue);
                        break;
                    case 'd':
                        // motorDriver.setD(commandValue);
                        break;
                    case 'z':                        
                        ESP_LOGE(TAG, "Starting PID tuning");
                        // motorDriver.startTuning();
                        break;
                    case 'w':                        
                        // ESP_LOGE(TAG, "Calibrate current response %.3f", motorDriver.calibrateCurrent(commandValue/100));
                        break;
                    case 's':
                        // motorDriver.logInfo(false);
                        // twai.logStatus();
                        break;
                }
                
                inputPos = 0;
            } else {
                inputBuffer[inputPos++] = ch;
            }
	    }                    
        
        mc.logInfo();
        vTaskDelay(CONFIG_PRINT_DELAY / portTICK_PERIOD_MS);
    }
}