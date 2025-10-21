#include "mt6701.h"
#include "utils.h"

class MT6701Emulator : public MT6701 {
    public:
        MT6701Emulator(double weight_kg, double arm_length_m) {
            load_inertia_factor = 1.0 + (weight_kg * arm_length_m * arm_length_m) * 10.0;
        }
    
        const float REVOLUTIONS_PER_SECOND = 10;
        void begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss) override;
        bool read(double *angle, MT6701::mt6701_status_t *field_status, bool *button_pushed, bool *track_loss) override;
        
        void setSpeedAndDirection(bool direction, uint16_t speed);

    private:
        void computePosition();
        uint32_t lastComputeMillis;
        float angle = 0;
        uint8_t speed = 0;
        bool direction = true;

        double current_velocity = 0;
        double current_position = 0;
        double load_inertia_factor = 1.0; // 1.0 = no load, higher = more inertia
};