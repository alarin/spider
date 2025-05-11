#include "ACS712.h"

#define ADC_ATTEN        ADC_ATTEN_DB_11  // 0-3.1V range
#define ADC_WIDTH        ADC_WIDTH_BIT_12 // 12-bit resolution

void ACS712::setup(adc1_channel_t pin) {
    _pin = pin;
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(_pin, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 0, &_adc_chars);
}

float ACS712::readCurrent() {
    uint32_t raw = _readRawAverage();
    
    // Convert to voltage (with calibration and divider compensation)
    float voltage = esp_adc_cal_raw_to_voltage(raw, &_adc_chars) / 1000.0;
    voltage *= VOLTAGE_DIVIDER_RATIO;

    // Calculate current
    float zero_voltage = esp_adc_cal_raw_to_voltage(ZERO_CURRENT_ADC, &_adc_chars) / 1000.0 * VOLTAGE_DIVIDER_RATIO;
    float current = (voltage - zero_voltage) / (MVA/1000.0);

    return current;
}

float ACS712::calibrate(float realCurrent) {
    if (abs(realCurrent) <= 0.0001) {
        //zero point calibration
        return _readRawAverage();
    } else {
        float measuredVoltage = readCurrent() * (MVA/1000.0);
        return measuredVoltage / realCurrent;
    }
}

uint32_t ACS712::_readRawAverage() {
    const int samples = 128; // Increased for better noise immunity
    uint32_t raw = 0;
    
    // Sample averaging
    for(int i=0; i<samples; i++) {
        raw += adc1_get_raw(_pin);
    }
    raw /= samples;
    return raw;    
}