#include "motordriver.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "utils.h"
#include "math.h"
#include "esp_timer.h"
#include <string.h>

#include "config.h"

void MotorDriver::setTargetAngle(double angle) {
    if (angle < _min_angle) {
        angle = _min_angle;
    }
    if (angle > _max_angle) {
        angle = _max_angle;
    }
    ESP_LOGI(TAG, "Set target angle to %lf", angle);
    setState(State::NORMAL);
    oscilationDetector.reset();
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

MotorDriver::State MotorDriver::getState() {
    return _state;
}

void MotorDriver::setup(double min_angle, double max_angle) {
    ESP_LOGI(TAG, "Setting up..");
    encoder.begin(PIN_MT6701_SCLK, PIN_MT6701_MISO, PIN_MT6701_CS);
    bool result = false;
    for (int i=0; i < 5; i++) {
        result = encoder.read(&_current_angle, NULL, NULL, NULL);    
    }
    if (!result) {        
        ESP_LOGE(TAG, "Encoder CRC ERROR, cannot detect current angle");
        _state = State::ENCODER_ERROR;
    }
    assert(result);

    _min_angle = min_angle;
    _max_angle = max_angle;
    setTargetAngle(_current_angle);

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
    positionPID.SetOutputLimits(-OUTPUT_MID_POINT, OUTPUT_MID_POINT);
    positionPID.SetMode(QuickPID::Control::automatic);

    ESP_LOGI(TAG, "Settings up tasks");
    // xTaskCreate(
    //     MotorDriver::computeTask,    // Function that should be called
    //     "Compute PID",   // Name of the task (for debugging)
    //     10000,            // Stack size (bytes)
    //     this,      // Parameter to pass
    //     10,               // Task priority
    //     NULL             // Task handle
    // );
    esp_timer_handle_t ctrl_timer;
    esp_timer_create_args_t args{
        .callback = [](void* arg){ static_cast<MotorDriver*>(arg)->compute(); },
        .arg = this, .name = "ctrl"
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &ctrl_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(ctrl_timer, SAMPLE_TIME_US));    
}

double MotorDriver::getCurrentAngle() {
    return _current_angle;
}

void MotorDriver::logInfo(bool skipSameLogs) {
    //char currentMessage[254];
    // sprintf(currentMessage, "_DD_ state: %d, angle: %.2f, target: %.2f, output: %.2f, duty: %.2f, current: %.2f", _state, _current_angle, _target_angle, _pid_output, _last_duty_cycle, _current);
    // if (!skipSameLogs || strcmp(currentMessage, lastLogMessage) != 0) {
    //     ESP_LOGI(TAG, "%s", currentMessage);
    //     strncpy(lastLogMessage, currentMessage, 254);
    // }    
    if (hand_tuning) {
        StabilityResult result = oscilationDetector.analyzeAmplitudeStability();   
        if (result.peak_data.period != 0) {
            ESP_LOGE(TAG, "Analysis complete: stable=%s, slope=%.4f, CV=%.4f, period=%.3fs",
                result.is_stable ? "true" : "false", 
                result.amplitude_slope, 
                result.amplitude_cv,
                result.peak_data.period);
            for(int i=0; i < result.peak_data.amplitudes.size(); i++) {
                ESP_LOGE(TAG, "%.2f", result.peak_data.amplitudes[i]);
            }
        }
        if (result.is_stable) {
            float Ku = positionPID.GetKp();
            float Tu = result.peak_data.period;       
            //positionPID.SetTunings(0.33 * Ku, 0.66 * Ku/Tu, 0.11 * Ku * Tu); //some overshoot
            positionPID.SetTunings(0.20 * Ku, 0.4 * Ku/Tu, 0.066 * Ku * Tu); //no overshoot
            hand_tuning = false;
        }
    }
    ESP_LOGI(TAG, "_DD_ state: %d, angle: %.2f, target: %.2f, output: %.2f, duty: %.2f, current: %.2f, p: %.2f, i: %.2f, d: %.2f,", 
        _state, _current_angle, _target_angle, _pid_output, _last_duty_cycle, _current, positionPID.GetKp(), positionPID.GetKi(), positionPID.GetKd());
}

void MotorDriver::startTuning() {
    // _tuning = 1;
    // _tuning_cycle = 0;
    // _tuning_max_angle = 0;
    // _tuning_min_angle = 360;
    // _min_angle_t = 0;
    // _max_angle_t = 0;    
    // positionPID.SetTunings(positionPID.GetKp(), 0, 0);
    // setTargetAngle(0);
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // setTargetAngle(100);
    hand_tuning = true;
}

/*
Step-by-step guide Configure the controller: 

Turn off the integral (I) and derivative (D) actions. This is often done by setting the integral time (\(T_{i}\)) to its maximum value and the derivative time (\(T_{d}\)) to zero.

Induce oscillations: Introduce a small disturbance in the loop, for example, by changing the setpoint slightly.

Find the ultimate gain (\(K_{u}\)): Increase the proportional gain (\(K_{p}\)) incrementally until the system output shows stable, sustained oscillations.

Measure the ultimate period (\(P_{u}\)): While the oscillations are stable, measure the time period of one full oscillation. This is the ultimate period (\(P_{u}\)).

Calculate PID parameters: Use the values of \(K_{u}\) and \(P_{u}\) and the Ziegler-Nichols tuning rules to calculate the final controller parameters.
https://en.wikipedia.org/wiki/Ziegler–Nichols_method
*/
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
                    _min_angle_t = Mmillis();
                }
                if (abs(_current_angle - _tuning_max_angle) <= 0.1) {
                    _max_angle_t = Mmillis();
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

void MotorDriver::setState(State new_state) {
    if (_state == new_state) {
        //do nothing
        return;
    }

    if (new_state == State::ENCODER_ERROR) {
        ESP_LOGE(TAG, "Encoder CRC ERROR, stopping");
    }

    if (new_state == State::MIN_MAX_ANGLE_PROTECTION) {
        ESP_LOGE(TAG, "Max or min angle protection, stopping %f", _current_angle);
    } 

    if (new_state == State::MAX_CURRENT_PROTECTION) {
        ESP_LOGE(TAG, "Too much current, stopping %f, limit %f", _current, MAX_CURRENT);
    }

    if (new_state == State::NORMAL) {
        ESP_LOGE(TAG, "Returned to normal state");
    }

    _state = new_state;

    if (_state != State::NORMAL) {
        gpio_set_level(PIN_MOTOR_BRAKE, true);
    } else {
        gpio_set_level(PIN_MOTOR_BRAKE, false);
    }
}


void MotorDriver::compute() {
    bool result = encoder.read(&_current_angle, NULL, NULL, NULL);    
    if (!result) {
        if (!encoder.read(&_current_angle, NULL, NULL, NULL)) {
            setState(State::ENCODER_ERROR);
        }
    }
    oscilationDetector.addData(_current_angle, Mmillis()/1000.0);

    if (_tuning > 0) {
        _tune_cycle();
    }

    positionPID.Compute();

    bool direction = (_pid_output > 0.f);
    float duty = fabsf(_pid_output) / OUTPUT_MID_POINT;
    if (duty < 0.12f) duty = 0.f;    

    if ((_current_angle <= _min_angle && !direction)
        || (_current_angle >= _max_angle && direction)) {
            setState(MIN_MAX_ANGLE_PROTECTION);
    }

    _current = currentSensor.readCurrent();
    if (_current >= MAX_CURRENT) {
        setState(MAX_CURRENT_PROTECTION);
    }

    if (_state == State::NORMAL) {
        gpio_set_level(PIN_MOTOR_DIR, direction);
        setMotorPWM(duty);
        encoder.setSpeedAndDirection(direction, abs(_pid_output));
    } else {
        setMotorPWM(0);
        encoder.setSpeedAndDirection(direction, 0);
    }    
} 

// void MotorDriver::computeTask(void *pvParameters) {
//     for(;;){
//         MotorDriver *l_pThis = (MotorDriver *) pvParameters;   
//         l_pThis->compute();
//         vTaskDelay(COMPUTE_TASK_DELAY_MS / portTICK_PERIOD_MS);
//     }
// }

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
    _last_duty_cycle = duty_cycle;
    const uint32_t max_duty = (1 << LEDC_TIMER_13_BIT) - 1; // 8191 for 13-bit
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle * max_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}