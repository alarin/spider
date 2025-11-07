#include <variant>

#include "mt6701.h"
#include "mt6701_emulator.h"
#include "ACS712.h"
#include "QuickPID.h"
#include "driver/gpio.h"

#include "config.h"
#include "oscilation_detector.h"

class MotorDriver {
    public:
        enum State {
            NORMAL,
            ENCODER_ERROR,
            MAX_CURRENT_PROTECTION,
            MIN_MAX_ANGLE_PROTECTION
        };

        MotorDriver(MT6701& enc) : 
            encoder(enc), 
            positionPID(&_current_angle, &_pid_output, &_target_angle, Kp, Ki, Kd, QuickPID::Action::direct),
            oscilationDetector() {
            _state = State::NORMAL;
        }
        
        void setup(double min_angle = 0, double max_angle = 300);

        void setTargetAngle(double angle);
        double getCurrentAngle();

        float getCurrent();

        void setP(double p);
        void setI(double i);
        void setD(double d);

        float calibrateCurrent(float realCurrent);

        void logInfo(bool skipSameLogs = true);

        void startTuning();

        State getState();

    private:
        static constexpr const char* TAG = "spider-servo-motor-driver";
        
        // static constexpr double Kp = 7.8;
        // static constexpr double Ki = 28.36;
        // static constexpr double Kd = 0.54;
        static constexpr double Kp = P;
        static constexpr double Ki = I;
        static constexpr double Kd = D;
        
        static constexpr uint32_t SAMPLE_TIME_US = 10 * 1000;
        static constexpr uint16_t TUNING_CYCLES = 500;
 
        static constexpr uint8_t OUTPUT_MID_POINT = 255;
        static constexpr float MAX_CURRENT = CURRENT_LIMIT;

        bool hand_tuning = false;

        uint8_t _tuning = 0;
        uint32_t _tuning_cycle = 0;
        float _tuning_max_angle = 0;
        uint32_t _max_angle_t = 0;
        float _tuning_min_angle = 360;
        uint32_t _min_angle_t = 0;
        float _last_duty_cycle = 0;

        State _state;
        float _min_angle;
        float _max_angle;
        
        float _current;
        float _current_angle;
        float _target_angle;
        float _pid_output;

        ACS712 currentSensor;
        MT6701 &encoder;
        QuickPID positionPID;
        AmplitudeStabilityAnalyzer oscilationDetector;

        //for logging not doubling
        char lastLogMessage[254];

        void setState(State new_state);
        void compute();
        void pwmInit();
        void setMotorPWM(float duty_cycle);       

        void _tune_cycle();
};