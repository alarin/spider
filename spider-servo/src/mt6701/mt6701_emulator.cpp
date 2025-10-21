#include "mt6701_emulator.h"
#include "esp_log.h"

void MT6701Emulator::begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss) {
    lastComputeMillis = Mmillis();
    angle = 50;
    current_position = 50;
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
        uint32_t millisFromLastCompute = Mmillis() - lastComputeMillis;
        
        double dt = millisFromLastCompute / 1000.0;
        
        // Convert command to target velocity
        double target_velocity = speed / (double)OUTPUT_MID_POINT * 360 * REVOLUTIONS_PER_SECOND;

        if (!direction) {
            target_velocity *= -1;
        }

        // Simple first-order filter - works for both positive and negative
        double alpha = dt / (0.1 * load_inertia_factor + dt); // Smoothing factor
        current_velocity = (1 - alpha) * current_velocity + alpha * target_velocity;
        
        // Update position
        current_position += current_velocity * dt;
        // if (current_position != angle) {
        //     ESP_LOGE("emu", "%ld %.2f", millisFromLastCompute, current_position);
        // }
        angle = current_position;
        if (angle < 0) {
            angle = 0;//TWO_PI - 0.000001;
        }
        if (angle > 360) {
            angle = 360;//0 + 0.000001;
        }
    }
    lastComputeMillis = Mmillis();
}