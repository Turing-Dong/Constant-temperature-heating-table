#include "./BSP/GLOBALS/globals.h"
#include "./SYSTEM/sys/sys.h"

#define SYS_NORMAL 1
#define SYS_WORKING 2

volatile int16_t enc_delta = 0;
volatile uint8_t sw_press = 0;
volatile uint8_t beep_req = 0;
volatile uint8_t update_req = 1;

int16_t temp_set = 250;
uint8_t SYS_STATE = SYS_NORMAL;


