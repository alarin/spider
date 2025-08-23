#include "mt6701.h"
#include "utils.h"

class MT6701Emulator : public MT6701 {
    public:
        const float REVOLUTIONS_PER_SECOND = 1;
        void begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss);
        bool read(double *angle, MT6701::mt6701_status_t *field_status, bool *button_pushed, bool *track_loss );
        void computePosition();
        void setSpeedAndDirection(bool direction, uint16_t speed);
    private:
        uint32_t lastComputeMillis;
        float angle = 0;
        uint8_t speed = 0;
        bool direction = true;
};