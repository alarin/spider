#include "motordriver.h"

#include "driver/ledc.h"
#include "esp_log.h"


#define ADC_ATTEN        ADC_ATTEN_DB_11  // 0-3.1V range
#define ADC_WIDTH        ADC_WIDTH_BIT_12 // 12-bit resolution
#define ZERO_CURRENT_ADC 2048            // Calibrate this value!
#define VOLTAGE_DIVIDER_RATIO 1.5        // 2k/(1k+2k) divider ratio


void MotorDriver::setTargetAngle(double angle) {
    _target_angle = angle;
}

void MotorDriver::setup(double min_angle, double max_angle) {
    encoder.begin(PIN_MT6701_SCLK, PIN_MT6701_MISO, PIN_MT6701_CS);

    gpio_reset_pin(PIN_MOTOR_DIR);
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_MOTOR_PWM);
    gpio_set_direction(PIN_MOTOR_PWM, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_MOTOR_BRAKE);
    gpio_set_direction(PIN_MOTOR_BRAKE, GPIO_MODE_OUTPUT);

    //curent sensor
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(PIN_CURRENT_SENSOR, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 0, &adc_chars);

    pwmInit();

    positionPID.SetSampleTimeUs(SAMPLE_TIME_US);
    positionPID.SetOutputLimits(0, OUTPUT_MID_POINT*2);
    positionPID.SetMode(QuickPID::Control::automatic);
}

float MotorDriver::readCurrent() {
    const int samples = 128; // Increased for better noise immunity
    uint32_t raw = 0;
    
    // Sample averaging
    for(int i=0; i<samples; i++) {
        raw += adc1_get_raw(PIN_CURRENT_SENSOR);
    }
    raw /= samples;

    // Convert to voltage (with calibration and divider compensation)
    float voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0;
    voltage *= VOLTAGE_DIVIDER_RATIO;

    // Calculate current (ACS712 20A: 100mV/A)
    float zero_voltage = esp_adc_cal_raw_to_voltage(ZERO_CURRENT_ADC, &adc_chars) / 1000.0 * VOLTAGE_DIVIDER_RATIO;
    float current = (voltage - zero_voltage) / 0.1;

    return current;
}

void MotorDriver::compute() {
    
    //read angle
    bool result = encoder.read(&_current_angle, NULL, NULL, NULL);
    if (!result) {
        ESP_LOGE(TAG, "CRC ERROR");
        setMotorPWM(0);
        return;
    }
    //test current
    float current = readCurrent();
    if (current >= 4) {
        ESP_LOGE(TAG, "Too much current, stopping %f", current);
        setMotorPWM(0);
        return;
    }
    //test angle min max


    positionPID.Compute();
    setSpeedAndDirection();
}

void MotorDriver::pwmInit() {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT, // 8192 steps (0-8191)
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000, // PWM frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t channel_cfg = {
        .gpio_num = PIN_MOTOR_PWM, // Your GPIO number
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Initial duty
        .hpoint = 0
    };
    ledc_channel_config(&channel_cfg);
}

void MotorDriver::setMotorPWM(float duty_cycle) {
    const uint32_t max_duty = (1 << LEDC_TIMER_13_BIT) - 1; // 8191 for 13-bit
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle * max_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void MotorDriver::setSpeedAndDirection() {
    bool direction;
    uint8_t speed;

    if (_pid_output > OUTPUT_MID_POINT) {
        direction = true;
        speed = _pid_output - OUTPUT_MID_POINT;
    } else {
        direction = false;
        speed = OUTPUT_MID_POINT - _pid_output - 2;
    }
    if ((speed) < 20) {
        speed = 0;
    }
    gpio_set_level(PIN_MOTOR_DIR, direction);
    setMotorPWM(speed);
}
