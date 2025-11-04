#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "config.h"
#include "motortwai.h"

static const char* TAG = "TEST_MOTOR_TWAI";

extern "C" {
    void app_main(void);
}

// Optional functions to set up and clean up after each test
void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void testSelfSend(void) {
    const float ANGLE = 100;

    MotorTWAI twai;
    twai.setup(LEG_ID, MOTOR_ID, TWAI_TX_PIN, TWAI_RX_PIN, true);
    motor_command_t cmd = {
        .command = SET_ANGLE,
        .param = ANGLE
    };
    twai.testSendMotorCommand(cmd);
    twai.logStatus();

    vTaskDelay(pdMS_TO_TICKS(100));

    motor_command_t newCmd;
    TEST_ASSERT_EQUAL(true, twai.receive(&newCmd));    
    TEST_ASSERT_EQUAL(cmd.command, newCmd.command);
    TEST_ASSERT_EQUAL(cmd.param, newCmd.param);

    motor_status_t status = {
        .state = MAX_CURRENT_PROTECTION,
        .angle = ANGLE
    };
    twai.sendStatus(status);
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(false, twai.receive(&newCmd)); //got filtered
}

// Main function to run all tests
void app_main() {
  // Brief delay for the chip to initialize
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  UNITY_BEGIN();
  RUN_TEST(testSelfSend);
  UNITY_END();
}