#include "mt6701.h"

class MT6701Emulator : public MT6701 {
    public:
        const float REVOLUTIONS_PER_SECOND = 0.1;
    
        bool read(float *angle, MT6701::mt6701_status_t *field_status, bool *button_pushed, bool *track_loss );
        void computePosition();
        void setSpeed(uint16_t speed);
    private:
        uint32_t lastComputeMillis = millis();
        float angle = 0;
        uint8_t speed = 0;
        bool direction = true;
};