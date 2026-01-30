#include "./BSP/ENCODER/encoder.h"
#include "./BSP/GLOBALS/globals.h"

static uint8_t enc_state = 0; // 当前A/B状态（0~3）

// 格雷码状态转移表：索引 = (prev_state << 2) | curr_state → delta
static const int8_t ENC_TABLE[16] = {
    0, +1, -1, 0,
    -1, 0, 0, +1,
    +1, 0, 0, -1,
    0, -1, +1, 0
};

void encoder_update(uint8_t a, uint8_t b)
{
    uint8_t curr = (a << 1) | b;
    uint8_t idx = (enc_state << 2) | curr;
    enc_delta += ENC_TABLE[idx];
    enc_state = curr;
}


