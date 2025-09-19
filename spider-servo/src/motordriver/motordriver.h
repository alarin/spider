#include "mt6701/mt6701.h"
#include "mt6701/mt6701_emulator.h"
#include "currentsensor/ACS712.h"
#include "QuickPid.h"
#include "driver/gpio.h"
#include "driver/adc.h"

class MotorDriver {
    public:
        enum State {
            NORMAL,
            ENCODER_ERROR,
            MAX_CURRENT_PROTECTION,
            MIN_MAX_ANGLE_PROTECTION
        };

        MotorDriver() : 
            encoder(), 
            positionPID(&_current_angle, &_pid_output, &_target_angle, Kp, Ki, Kd, QuickPID::Action::direct) {
            _state = State::NORMAL;
        }
        
        void setup(double min_angle = 0, double max_angle = 300);
        static void computeTask(void *pvParameters);

        void setTargetAngle(double angle);
        double getCurrentAngle();

        void setP(double p);
        void setI(double i);
        void setD(double d);

        float calibrateCurrent(float realCurrent);

        void logInfo();

        void startTuning();

        State getState();

    private:
        static constexpr const char* TAG = "spider-servo-motor-driver";
        
        static constexpr double Kp = 7.8;
        static constexpr double Ki = 28.36;
        static constexpr double Kd = 0.54;
        static constexpr uint32_t SAMPLE_TIME_US = 10 * 1000;
        static constexpr uint32_t COMPUTE_TASK_DELAY_MS = 10;
        static constexpr uint16_t TUNING_CYCLES = 500;
 
        static constexpr uint8_t OUTPUT_MID_POINT = 255;
        static constexpr double MAX_CURRENT = 0.5;

        static constexpr adc1_channel_t PIN_CURRENT_SENSOR = ADC1_CHANNEL_0;
        static constexpr gpio_num_t PIN_MOTOR_PWM = GPIO_NUM_1;
        static constexpr gpio_num_t PIN_MOTOR_DIR = GPIO_NUM_2;
        static constexpr gpio_num_t PIN_MOTOR_BRAKE = GPIO_NUM_3;
        
        static constexpr gpio_num_t PIN_MT6701_MISO = GPIO_NUM_8;
        static constexpr gpio_num_t PIN_MT6701_SCLK = GPIO_NUM_9;
        static constexpr gpio_num_t PIN_MT6701_CS = GPIO_NUM_10;

        static constexpr double ANGLE_PROTECTION_DIFF = 3;


        uint8_t _tuning = 0;
        uint32_t _tuning_cycle = 0;
        double _tuning_max_angle = 0;
        uint32_t _max_angle_t = 0;
        double _tuning_min_angle = 360;
        uint32_t _min_angle_t = 0;

        State _state;
        double _min_angle = 115;
        double _max_angle = 290;
        
        double _current;
        double _current_angle;
        double _target_angle;
        double _pid_output;

        ACS712 currentSensor;
        MT6701 encoder;
        QuickPID positionPID;
        
        void compute();
        void pwmInit();
        void setMotorPWM(float duty_cycle);       
        float readCurrent();

        void setSpeedAndDirection();
        void _tune_cycle();
};