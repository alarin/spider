#include "motordriver.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "utils.h"


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

void MotorDriver::startTuning() {
    _tuning = 1;
    _tuning_cycle = 0;
    _tuning_max_angle = 0;
    _tuning_min_angle = 360;
    _min_angle_t = 0;
    _max_angle_t = 0;    
    positionPID.SetTunings(positionPID.GetKp(), 0, 0);
    setTargetAngle(100);
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
        10,               // Task priority
        NULL             // Task handle
    );        
}

double MotorDriver::getCurrentAngle() {
    return _current_angle;
}

void MotorDriver::logInfo() {
    ESP_LOGI(TAG, "_DD_ state: %d, angle: %.2f, target: %.2f, output: %.2f, current: %.2f, p %.2f, i %.2f, d %.2f", 
        _state, _current_angle, _target_angle, _pid_output, _current, positionPID.GetKp(), positionPID.GetKi(), positionPID.GetKd());
}

void MotorDriver::_tune_cycle() {
    if (_tuning > 0) {
        if (_tuning_cycle >= TUNING_CYCLES) {
            if (_tuning == 1) {
                ESP_LOGE(TAG, "Tuning: found min/max angles (%.2f, %.2f) calcuating Tu", _tuning_min_angle, _tuning_max_angle);
                _tuning = 2;
            }
            if (_min_angle_t != 0 && _max_angle_t != 0) {            
                double ku = positionPID.GetKp();
                double tu = abs((int64_t) _min_angle_t - (int64_t) _max_angle_t)/1000.0;
                double p = 0.6 * ku;
                double i = 1.2 * ku/tu;
                double d = 0.075 * ku * tu;
                ESP_LOGE(TAG, "Tuning finished Ku %.2f, Tu %.4f, p %.4f, i %.4f, d %.4f", ku, tu, p, i, d);
                positionPID.SetTunings(p, i, d);
                _tuning = 0;
            } else {
                if (abs(_current_angle - _tuning_min_angle) <= 0.1) {
                    _min_angle_t = millis();
                }
                if (abs(_current_angle - _tuning_max_angle) <= 0.1) {
                    _max_angle_t = millis();
                }
            }
        } else if (_tuning_cycle >= TUNING_CYCLES/2) {
            if (_current_angle < _tuning_min_angle) {
                _tuning_min_angle = _current_angle;
            } 
            if (_current_angle > _tuning_max_angle) {
                _tuning_max_angle = _current_angle;
            }
        }
        _tuning_cycle++;
    }
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

    if (_tuning > 0) {
        _tune_cycle();
    }

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
        vTaskDelay(COMPUTE_TASK_DELAY_MS / portTICK_PERIOD_MS);
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
    if ((speed) < 30) {
        speed = 0;
    }
    // encoder.setSpeedAndDirection(direction, speed);
    gpio_set_level(PIN_MOTOR_DIR, direction);

    // if ((_current_angle <= _min_angle && !direction)
    //     || (_current_angle >= _max_angle && direction)) {
    //     ESP_LOGE(TAG, "Max or min angle protection, stopping %f", _current_angle);
    //     speed = 0;        
    // }
    setMotorPWM(speed);
}
