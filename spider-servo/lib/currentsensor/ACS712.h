#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class ACS712 {
    public:
        void setup(adc_channel_t pin);
        float readCurrent();

        float calibrate(float realCurrent);
    private:
        static constexpr double VOLTAGE_DIVIDER_RATIO = 1.5;
        static constexpr int ZERO_CURRENT_ADC = 1872;
        //ACS712 20A 100mV/A
        //ACS712 5A 185mV/A 
        static constexpr int MVA = 220;//100;
        

        adc_channel_t _pin;
        adc_oneshot_unit_handle_t _adc_handle;
        adc_cali_handle_t _adc_cali_handle;
        int _zero_voltage;

        uint32_t _readRawAverage();
};
