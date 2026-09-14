#ifndef __DR16_H
#define __DR16_H

#include "main.h"

#define RC_CH_VALUE_MIN         ((int16_t)-660)
#define RC_CH_VALUE_OFFSET      ((int16_t)1024)
#define RC_CH_VALUE_MAX         ((int16_t)660)

#define RC_SW_UP                ((int8_t)1)
#define RC_SW_MID               ((int8_t)3)
#define RC_SW_DOWN              ((int8_t)2)

typedef struct {
    struct {
        int16_t ch[5]; // ch[0]~ch[3]为双摇杆通道, ch[4]为左上角侧边拨轮 (-660 ~ 660)
        int8_t s[2];   // s[0]为左侧开关S1, s[1]为右侧开关S2 (1:上, 3:中, 2:下)
    } rc;
} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

void DR16_UART_Init(void);
void DR16_Decode(volatile const uint8_t *buff);

#endif
