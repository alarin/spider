#include "twai_proto.h"

uint32_t createMsgId(TwaiCommand cmd, uint8_t legN, uint8_t motorN) {
    return cmd << 6 | ((legN & 0b111) << 3) | (motorN & 0b111);
}

twai_filter_config_t createMotorFilter(uint8_t legN, uint8_t motorN) {
    return {.acceptance_code = uint32_t(createMsgId(SET_MOTOR, legN, motorN) << 21),
                                        .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
                                        .single_filter = true};
}
