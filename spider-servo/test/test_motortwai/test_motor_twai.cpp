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


void testLoopback(void) {
    static constexpr float ANGLE = 111;

    MotorTWAI twai;
    
    twai.setup(LEG_ID, MOTOR_ID, TWAI_TX_PIN, TWAI_RX_PIN, true);
    motor_status_t mstatus = {
      .state = MAX_CURRENT_PROTECTION,
      .angle = ANGLE
    };
    twai.sendStatus(mstatus);

    motor_command_t recievedMcmd;
    TEST_ASSERT_EQUAL(false, twai.rxFromQueue(&recievedMcmd, pdMS_TO_TICKS(100))); //should be filtered

    motor_command_t sentMcmd = {
      .command = SET_ANGLE,
      .param = ANGLE
    };
    twai.sendMotorCommand(sentMcmd);

    TEST_ASSERT_EQUAL(true, twai.rxFromQueue(&recievedMcmd, pdMS_TO_TICKS(100)));
    ESP_LOG_BUFFER_HEX(TAG, &recievedMcmd, sizeof(recievedMcmd));
    
    twai.logStatus();

    TEST_ASSERT_EQUAL(sentMcmd.command, recievedMcmd.command);
    TEST_ASSERT_EQUAL(sentMcmd.param, recievedMcmd.param);
}

void app_main() {
  // Brief delay for the chip to initialize
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  UNITY_BEGIN();
  RUN_TEST(testLoopback);
  UNITY_END();
}