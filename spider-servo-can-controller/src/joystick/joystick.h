#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include <stdlib.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class Joystick {
private:
    static const int _x_right_threshold = 2500;
    static const int _x_left_threshold = 2000;
    adc_oneshot_unit_handle_t _adc_x_handle;
    adc_cali_handle_t _adc_cali_handle;
    int _x;
    void _readInputs();
public:
    void setup();
    int8_t getX();
    
};

#endif
