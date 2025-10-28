#include "ACS712.h"

#define ADC_UNIT ADC_UNIT_1
#define ATTEN ADC_ATTEN_DB_12

void ACS712::setup(adc_channel_t pin) {
    _pin = pin;
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &_adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc_handle, _pin, &config));
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &_adc_cali_handle));
    adc_cali_raw_to_voltage(_adc_cali_handle, ZERO_CURRENT_ADC, &_zero_voltage);
}

float ACS712::readCurrent() {
    uint32_t raw = _readRawAverage();
    
    // Convert to voltage (with calibration and divider compensation)
    int voltage;
    adc_cali_raw_to_voltage(_adc_cali_handle, raw, &voltage);

    float current = (voltage - _zero_voltage)/1000.0 * VOLTAGE_DIVIDER_RATIO / (MVA/1000.0);

    return current;
}

float ACS712::calibrate(float realCurrent) {
    if (abs(realCurrent) <= 0.0001) {
        //zero point calibration
        return _readRawAverage();
    } else {
        float measuredVoltage = readCurrent() * (MVA/1000.0) / VOLTAGE_DIVIDER_RATIO;
        return measuredVoltage / realCurrent * 1000;
    }
}

uint32_t ACS712::_readRawAverage() {
    const int samples = 128;
    uint32_t raw = 0;
    int raw_sample = 0;
    
    // Sample averaging
    for(int i=0; i < samples; i++) {
        adc_oneshot_read(_adc_handle, _pin, &raw_sample);
        raw += raw_sample;
    }
    raw /= samples;
    return raw;    
}