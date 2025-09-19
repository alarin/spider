#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

class Joystick {
private:
    static const uint32_t _x_right_threshold = 3750;
    static const uint32_t _x_left_threshold = 3000;
    uint32_t _x;
    void _readInputs();
public:
    void setup();
    int8_t getX();
    
};

#endif
