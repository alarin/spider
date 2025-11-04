#include <unity.h>
#include <functional>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "twai_proto.h"
#include "motortwai.h"
#include "config.h"

static const char* TAG = "TEST_OSC_DETECT";

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

static motor_command_t receivedCommand;

static void motorCommandReceived(motor_command_t cmd) {
    receivedCommand = cmd;
}

void testLoopback(void) {
    static constexpr float ANGLE = 111;

    MotorTWAI twai;
    twai.setOnMotorCommandCallback(motorCommandReceived);
    twai.setup(LEG_ID, MOTOR_ID, TWAI_TX_PIN, TWAI_RX_PIN, false);
    motor_status_t mstatus = {
      .state = MAX_CURRENT_PROTECTION,
      .angle = ANGLE
    };
    twai.sendStatus(mstatus);

    motor_command_t mcmd = {
      .command = SET_ANGLE,
      .param = ANGLE
    };
    twai.sendMotorCommand(mcmd);
    twai.sendMotorCommand(mcmd);

    vTaskDelay(pdMS_TO_TICKS(2000));
    twai.logStatus();

    TEST_ASSERT_EQUAL(mcmd.command, receivedCommand.command);
    TEST_ASSERT_EQUAL(mcmd.param, receivedCommand.param);
}

void app_main() {
  // Brief delay for the chip to initialize
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  UNITY_BEGIN();
  RUN_TEST(testLoopback);
  UNITY_END();
}