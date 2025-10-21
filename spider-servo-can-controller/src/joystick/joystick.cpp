#include "joystick.h"

void Joystick::setup() {
    // adc_oneshot_config_width(ADC_WIDTH_BIT_12); // Set 12-bit resolution
    // adc_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // GPIO34, 0-2450 mV range

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &_adc_x_handle));    
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,        
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc_x_handle, ADC_CHANNEL_0, &config));    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &_adc_cali_handle));
}

int8_t Joystick::getX() {
    _readInputs();
    if (_x >= _x_right_threshold) {
        ESP_LOGI("j", "%d > %d", _x, _x_right_threshold);
        return -1;
    } else if (_x <= _x_left_threshold) {
        ESP_LOGI("j", "%d < %d", _x, _x_left_threshold);
        return 1;
    } else {
        return 0;
    }
}

void Joystick::_readInputs() {
    // esp_adc_cal_characteristics_t adc_chars;
    // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 0, &adc_chars);
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(_adc_x_handle, ADC_CHANNEL_0, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_adc_cali_handle, raw, &_x));
    // uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(raw_value, &adc_chars);
    //ESP_LOGI(TAG, "%lud %lud", raw_value, voltage_mV);
}