#include "mt6701_emulator.h"
#include "esp_log.h"

void MT6701Emulator::begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss) {
    lastComputeMillis = millis();
}

bool MT6701Emulator::read(double *angle, MT6701::mt6701_status_t *field_status, bool *button_pushed, bool *track_loss ){
    computePosition();
    //emulate delay/interrupts whatever
    //readData();
    *angle = this->angle;
    return true;
}

#define OUTPUT_MID_POINT 255

void MT6701Emulator::setSpeedAndDirection(bool setDirection, uint16_t setSpeed) {
    direction = setDirection;
    speed = setSpeed;
}

void MT6701Emulator::computePosition() {
    if (lastComputeMillis > 0) {
        uint32_t millisFromLastCompute = millis() - lastComputeMillis;        

        double angle_diff = speed/(double)OUTPUT_MID_POINT * 360 * REVOLUTIONS_PER_SECOND/1000.0 * millisFromLastCompute;
        if (direction) {
            angle += angle_diff;
        } else {
            angle -= angle_diff;
        }
        // if (angle_diff) {
        //     ESP_LOGE("emu", "%ld %.2f", millisFromLastCompute, angle_diff);
        // }
        if (angle < 0) {
            angle = 0;//TWO_PI - 0.000001;
        }
        if (angle > 360) {
            angle = 360;//0 + 0.000001;
        }
    }
    lastComputeMillis = millis();
}