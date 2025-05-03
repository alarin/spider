#include "mt6701_emulator.h"

bool MT6701Emulator::read(float *angle, MT6701::mt6701_status_t *field_status, bool *button_pushed, bool *track_loss ){
    //emulate delay/interrupts whatever
    readData();
    *angle = this->angle;
    return true;
}

#define OUTPUT_MID_POINT 255

void MT6701Emulator::setSpeed(uint16_t newSpeed) {
    
    if (newSpeed > OUTPUT_MID_POINT) {
        direction = true;
        speed = newSpeed - OUTPUT_MID_POINT;
    } else {
        direction = false;
        speed = OUTPUT_MID_POINT - newSpeed - 1;
    }
    // speed = speed * 2;
    if ((speed) < 20) {
        speed = 0;
    }
}

void MT6701Emulator::computePosition() {
    if (lastComputeMillis > 0) {
        uint32_t millisFromLastCompute = millis() - lastComputeMillis;        

        float angle_diff = speed/(float)OUTPUT_MID_POINT * TWO_PI * REVOLUTIONS_PER_SECOND/1000.0 * millisFromLastCompute;
        if (direction) {
            angle += angle_diff;
        } else {
            angle -= angle_diff;
        }
        // if (angle_diff > 0) {
        //     Serial.print(angle_diff);
        //     Serial.print(" ");
        //     Serial.println(angle);
        // }        
        if (angle < 0) {
            angle = 0;//TWO_PI - 0.000001;
        }
        if (angle > TWO_PI) {
            angle = TWO_PI;//0 + 0.000001;
        }
    }
    lastComputeMillis = millis();
}