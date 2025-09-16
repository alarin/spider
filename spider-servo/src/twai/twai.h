#ifndef _TWAI_H_
#define _TWAI_H_

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "twai_proto.h"


class MOTOR_TWAI {
    public:
        void setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin = GPIO_NUM_21, gpio_num_t rx_pin = GPIO_NUM_20);
        bool receive(motor_command_t *cmd);
        void sendStatus(motor_status_t status);
        void logStatus();

    private:
        static constexpr const char* TAG = "TWAI";
        uint8_t _legn;
        uint8_t _motorn;
};

#endif