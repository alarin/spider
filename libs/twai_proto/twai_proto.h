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

#pragma pack(push, 1)
enum TwaiCommand {
    BROADCAST,
    MOTOR_COMMAND,
    MOTOR_FEEDBACK
};

enum TwaiMotorCommand {
    SET_ANGLE,
    REQUEST_STATUS
};

typedef struct {
    TwaiMotorCommand command;
    float param;
} motor_command_t;

enum MotorState {
    NORMAL,
    ENCODER_ERROR,
    MAX_CURRENT_PROTECTION,
    MIN_MAX_ANGLE_PROTECTION
};

typedef struct {
  MotorState state;
  float angle;
} motor_status_t;
#pragma pack(pop)
static_assert(sizeof(motor_command_t) <= 8, "TWAI payload too big");
static_assert(sizeof(motor_status_t)  <= 8, "TWAI payload too big");

uint32_t createMsgId(TwaiCommand cmd, uint8_t legN, uint8_t motorN);
twai_filter_config_t createMotorFilter(uint8_t legN, uint8_t motorN);

#endif