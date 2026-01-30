#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "./SYSTEM/sys/sys.h"

extern volatile int16_t enc_delta;   // 编码器变化量（+1/-1），主循环消费
extern volatile uint8_t sw_press;    // 按键按下事件（单次有效）
extern volatile uint8_t beep_req;    // 请求蜂鸣器响（主循环触发）
extern volatile uint8_t update_req;  // 请求刷新显示（主循环处理）

// 主循环中使用的业务变量（非 volatile，但访问需保护）
extern int16_t temp_set;
extern uint8_t SYS_STATE;

#endif



