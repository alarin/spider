#include "motordriver.h"

#include "driver/ledc.h"
#include "esp_log.h"


void MotorDriver::setTargetAngle(double angle) {
    // if (angle < _min_angle) {
    //     angle = _min_angle;
    // }
    // if (angle > _max_angle) {
    //     angle = _max_angle;
    // }
    _target_angle = angle;
}

void MotorDriver::setP(double p) {
    positionPID.SetTunings(p, positionPID.GetKi(), positionPID.GetKd());
}

void MotorDriver::setI(double i) {
    positionPID.SetTunings(positionPID.GetKp(), i, positionPID.GetKd());
}

void MotorDriver::setD(double d) {
    positionPID.SetTunings(positionPID.GetKp(), positionPID.GetKi(), d);
}

float MotorDriver::calibrateCurrent(float realCurrent) {
    return currentSensor.calibrate(realCurrent);
}

void MotorDriver::setup(double min_angle, double max_angle) {
    ESP_LOGI(TAG, "Setting up..");
    encoder.begin(PIN_MT6701_SCLK, PIN_MT6701_MISO, PIN_MT6701_CS);
    bool result = encoder.read(&_current_angle, NULL, NULL, NULL);
    if (!result) {        
        ESP_LOGE(TAG, "Encoder CRC ERROR, stopping");
        _state = State::ENCODER_ERROR;
    }
    _target_angle = _current_angle;

    ESP_LOGI(TAG, "Setting up, pins..");
    gpio_reset_pin(PIN_MOTOR_DIR);
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_MOTOR_PWM);
    gpio_set_direction(PIN_MOTOR_PWM, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_MOTOR_BRAKE);
    gpio_set_direction(PIN_MOTOR_BRAKE, GPIO_MODE_OUTPUT);

    //curent sensor
    ESP_LOGI(TAG, "Setting up current sensor");
    currentSensor.setup(PIN_CURRENT_SENSOR);
    
    ESP_LOGI(TAG, "Setting up motor pwm");
    pwmInit();
    setMotorPWM(0);

    positionPID.SetSampleTimeUs(SAMPLE_TIME_US);
    positionPID.SetOutputLimits(0, OUTPUT_MID_POINT*2);
    positionPID.SetMode(QuickPID::Control::automatic);

    ESP_LOGI(TAG, "Settings up tasks");
    xTaskCreate(
        MotorDriver::computeTask,    // Function that should be called
        "Compute PID",   // Name of the task (for debugging)
        10000,            // Stack size (bytes)
        this,      // Parameter to pass
        1,               // Task priority
        NULL             // Task handle
    );        
}

double MotorDriver::getCurrentAngle() {
    return _current_angle;
}

void MotorDriver::logInfo() {
    ESP_LOGI(TAG, "state: %d, angle: %.2f, target: %.2f, output: %.2f, current: %.2f", 
        _state, _current_angle, _target_angle, _pid_output, _current);
}

void MotorDriver::compute() {
    //read angle
    bool result = encoder.read(&_current_angle, NULL, NULL, NULL);
    if (!result) {        
        if (_state != State::ENCODER_ERROR) {
            ESP_LOGE(TAG, "Encoder CRC ERROR, stopping");
            _state = State::ENCODER_ERROR;
        }
        setMotorPWM(0);
        return;
    }
    _state = State::NORMAL;

    //test current
    _current = currentSensor.readCurrent();
    // if (current >= 4) {
    //     ESP_LOGE(TAG, "Too much current, stopping %f", current);
    //     setMotorPWM(0);
    //     return;
    // }
    positionPID.Compute();
    setSpeedAndDirection();    
} 

void MotorDriver::computeTask(void *pvParameters) {
    for(;;){
        MotorDriver *l_pThis = (MotorDriver *) pvParameters;   
        l_pThis->compute();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
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

    // if ((_current_angle <= _min_angle && !direction)
    //     || (_current_angle >= _max_angle && direction)) {
    //     ESP_LOGE(TAG, "Max or min angle protection, stopping %f", _current_angle);
    //     speed = 0;        
    // }
    setMotorPWM(speed);
}
