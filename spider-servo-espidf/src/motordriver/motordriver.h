#include "mt6701/mt6701.h"
#include "QuickPid.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

class MotorDriver {
    public:
        MotorDriver() : encoder(), positionPID(&_current_angle, &_pid_output, &_target_angle, Kp, Ki, Kd, QuickPID::Action::direct) {}
        
        void setup(double min_angle = 10, double max_angle = 200);
        void compute();

        void setTargetAngle(double angle);

    private:
        static constexpr char* TAG = "spider-servo-motor-driver";

        static constexpr double Kp = 10;
        static constexpr double Ki = 0.01;
        static constexpr double Kd = 0.5;
        static constexpr uint32_t SAMPLE_TIME_US = 100;

        static constexpr uint8_t OUTPUT_MID_POINT = 255;
        static constexpr uint8_t MAX_CURRENT = 4;

        static constexpr adc1_channel_t PIN_CURRENT_SENSOR = ADC1_CHANNEL_0;
        static constexpr gpio_num_t PIN_MOTOR_PWM = GPIO_NUM_1;
        static constexpr gpio_num_t PIN_MOTOR_DIR = GPIO_NUM_2;
        static constexpr gpio_num_t PIN_MOTOR_BRAKE = GPIO_NUM_3;
        
        static constexpr gpio_num_t PIN_MT6701_MISO = GPIO_NUM_5;
        static constexpr gpio_num_t PIN_MT6701_SCLK = GPIO_NUM_4;
        static constexpr gpio_num_t PIN_MT6701_CS = GPIO_NUM_7;

        double _min_angle;
        double _max_angle;
        
        double _current_angle;
        double _target_angle;
        double _pid_output;        
        esp_adc_cal_characteristics_t adc_chars;

        MT6701 encoder;
        QuickPID positionPID;
        
        void pwmInit();
        void setMotorPWM(float duty_cycle);        
        float readCurrent();

        void setSpeedAndDirection();
};