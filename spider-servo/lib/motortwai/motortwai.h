#ifndef _TWAI_H_
#define _TWAI_H_

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "twai_proto.h"


class MotorTWAI {
    public:
        void setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin);
        bool receive(motor_command_t *cmd);
        void sendStatus(motor_status_t status);
        void logStatus();

    private:
        static constexpr const char* TAG = "MTR_TWAI";
        uint8_t _legn;
        uint8_t _motorn;
};

#endif