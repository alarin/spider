#ifndef _TWAI_PROTO_H_
#define _TWAI_PROTO_H_

#include <stdlib.h>
#include "driver/twai.h"

/*
# 11 bit TWAI message id

each of 18 motors have personal msg_id

  cmd  leg  motor 
00000  000  000

# SET_MOTOR command 5 byte
byte 0 command (can set angle only now)
byte 1,2,3,4 param (float angle)

*/

enum TwaiCommand {
    BROADCAST,
    SET_MOTOR,
    MOTOR_FEEDBACK
};

typedef struct {
    uint8_t command;
    float param;
} motor_commant_t;


uint32_t createMsgId(TwaiCommand cmd, uint8_t legN, uint8_t motorN);
twai_filter_config_t createMotorFilter(uint8_t legN, uint8_t motorN);

#endif