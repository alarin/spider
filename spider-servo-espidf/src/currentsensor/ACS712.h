#include "driver/adc.h"
#include "esp_adc_cal.h"

class ACS712 {
    public:
        void setup(adc1_channel_t pin);
        float readCurrent();

        float calibrate(float realCurrent);
    private:
        static constexpr double VOLTAGE_DIVIDER_RATIO = 1.5;
        static constexpr int ZERO_CURRENT_ADC = 3629;
        //ACS712 20A 100mV/A
        //ACS712 5A 185mV/A 
        static constexpr int MVA = 348;//100;
        
        adc1_channel_t _pin;
        esp_adc_cal_characteristics_t _adc_chars;

        uint32_t _readRawAverage();
};
