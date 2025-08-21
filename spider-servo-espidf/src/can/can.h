#ifndef _CAN_H_
#define _CAN_H_

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/twai.h"

class CAN {
    public:
        void setup(gpio_num_t tx_pin = GPIO_NUM_20, gpio_num_t rx_pin = GPIO_NUM_21);

    private:
        static constexpr char* TAG = "spider-servo-can";
};

#endif