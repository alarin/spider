#include "joystick.h"

void Joystick::setup() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Set 12-bit resolution
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // GPIO34, 0-2450 mV range
}

int8_t Joystick::getX() {
    _readInputs();
    if (_x >= _x_right_threshold) {
        ESP_LOGI("j", "%lud > %lud", _x, _x_right_threshold);
        return -1;
    } else if (_x <= _x_left_threshold) {
        ESP_LOGI("j", "%lud < %lud", _x, _x_left_threshold);
        return 1;
    } else {
        return 0;
    }
}

void Joystick::_readInputs() {
    // esp_adc_cal_characteristics_t adc_chars;
    // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 0, &adc_chars);
    _x = adc1_get_raw(ADC1_CHANNEL_0);
    // uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(raw_value, &adc_chars);
    //ESP_LOGI(TAG, "%lud %lud", raw_value, voltage_mV);
}