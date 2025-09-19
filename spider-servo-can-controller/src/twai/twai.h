#ifndef _TWAI_H_
#define _TWAI_H_

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "twai_proto.h"


class TWAI {
    public:
        void setup(gpio_num_t tx_pin = GPIO_NUM_20, gpio_num_t rx_pin = GPIO_NUM_21);

        void logStatus();

        void sendAngle(uint8_t legn, uint8_t motorn, float angle);
        void requestStatus(uint8_t legn, uint8_t motorn);

        bool receive(motor_status_t *motor_status, uint32_t ticks = 100);
    private:
        static constexpr const char* TAG = "TWAI";
};

#endif