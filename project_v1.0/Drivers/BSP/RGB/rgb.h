#ifndef __RGB_H__
#define __RGB_H__

#include "./SYSTEM/sys/sys.h"

//WS2812B IO 定义  注意初始化APB时钟也要修改
#define WS2812B_PORT		GPIOA
#define WS2812B_PIN			GPIO_PIN_9

#define WS2812B_RCC_AHB()            do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define WS2812B_Hi()		WS2812B_PORT->BSRR=WS2812B_PIN	//GPIO_ResetBits(LED1_PORT, LED1_PIN)
#define WS2812B_Low()		WS2812B_PORT->BRR=WS2812B_PIN	//GPIO_SetBits(LED1_PORT, LED1_PIN)

//颜色
typedef struct color{
	u8 G;
	u8 R;
	u8 B;
}Color_TypeDef;

//------------------------
#define PIXEL_NUM 27	//LED灯的个数

extern Color_TypeDef PixelBuf[PIXEL_NUM];

void WS2812B_Init(void);
void WS2812B_Reset(void);
void WS2812B_WriteColor(Color_TypeDef *pColor);
void WS2812B_RefreshPixel(void);

void WS2812B_FillColor(u16 start,u16 end,Color_TypeDef *pColor);
void WS2812B_MovePixel(u8 dir);

void WS2812B_Test(void);
//测试延时时间
void WS2812B_Test2(void);

void WS2812B_SetPixelColor(uint16_t index, Color_TypeDef *color);

#endif


