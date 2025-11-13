#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/RGB/rgb.h"
#include "./BSP/LED/led.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/LCD/lcd_init.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/pic.h"
#include "./BSP/FAN/fan.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/TIM/gtim.h"
#include "./BSP/ADC/adc.h"
#include <math.h>

#define SYS_NORMAL 1
#define SYS_WORKING 2

// 在全局变量区域添加NTC参数 (根据实际测量值修正)
#define NTC_BETA 3950           // NTC热敏电阻的B值
#define NTC_R25 92180           // 25°C时的电阻值(根据实际测量为92.18K)
#define NTC_T25 298.15          // 25°C对应的开尔文温度

#define TEMP_SMOOTHING_SIZE 10  // 平滑窗口大小
double temp_buffer[TEMP_SMOOTHING_SIZE] = {0};  // 温度缓冲区
uint8_t temp_buffer_index = 0;                  // 缓冲区索引
uint8_t temp_buffer_full = 0;                   // 缓冲区是否已满标志

// 在全局变量区域添加RGB状态机相关变量
typedef enum {
    RGB_STATE_RAINBOW,      // 彩虹流水灯状态
    RGB_STATE_BLUE,         // 冷蓝色色状态 (加热时温度<40°C)
    RGB_STATE_YELLOW_GREEN, // 黄绿色色状态 (40-70°C)
    RGB_STATE_YELLOW,       // 黄色状态 (70-100°C)
    RGB_STATE_ORANGE,       // 橙色状态 (100-150°C)
    RGB_STATE_RED,          // 红色状态 (150-200°C)
    RGB_STATE_PURPLE        // 紫色状态 (>200°C)
} RGB_State_TypeDef;

RGB_State_TypeDef rgb_current_state = RGB_STATE_RAINBOW;  // 当前RGB状态
uint8_t rgb_state_changed = 1;  // 状态改变标志，初始设为1以触发首次写入

//数据表
//const unsigned int data_x[101] = 
//{
//	  0,  3,  6,  9, 12, 15, 18, 21, 24, 27,
//	 30, 33, 36, 39, 42, 45, 48, 51, 54, 57,
//	 60, 63, 66, 69, 72, 75, 78, 81, 84, 87,
//	 90, 93, 96, 99,102,105,108,111,114,117,
//	120,123,126,129,132,135,138,141,144,147,
//	150,153,156,159,162,165,168,171,174,177,
//	180,183,186,189,192,195,198,201,204,207,
//	210,213,216,219,222,225,228,231,234,237,
//	240,243,246,249,252,255,258,261,264,267,
//	270,273,276,279,282,285,288,291,294,297,
//	300
//};
//const float data_y[101] =
//{
//	325.0790,280.4187,242.2063,209.5278,181.5805,157.6669,137.1862,119.6241,104.5422, 91.5683,
//	 80.3868, 70.7309, 62.3750, 55.1284, 48.8297, 43.3428, 38.5520, 34.3596, 30.6826, 27.4504,
//	 24.6029, 22.0888, 19.8645, 17.8924, 16.1403, 14.5808, 13.1898, 11.9470, 10.8346,  9.8295,
//	  8.9273,  8.1205,  7.3983,  6.7509,  6.1697,  5.6471,  5.1764,  4.7514,  4.3669,  4.0183,
//	  3.7015,  3.4127,  3.1442,  2.8908,  2.6606,  2.4517,  2.2623,  2.0906,  1.9348,  1.7934,
//	  1.6649,  1.5480,  1.4415,  1.3441,  1.2551,  1.1733,  1.0981,  1.0286,  0.9643,  0.9016,
//	  0.8423,  0.7877,  0.7375,  0.6913,  0.6489,  0.6100,  0.5742,  0.5412,  0.5108,  0.4827,
//	  0.4566,  0.4324,  0.4099,  0.3887,  0.3689,  0.3502,  0.3312,  0.3134,  0.2966,  0.2809,
//	  0.2662,  0.2524,  0.2395,  0.2275,  0.2162,  0.2056,  0.1957,  0.1864,  0.1776,  0.1694,
//	  0.1616,  0.1542,  0.1473,  0.1407,  0.1345,  0.1285,  0.1228,  0.1174,  0.1122,  0.1071,
//	  0.10233
//};

struct struct_adc_data
{
	uint32_t adc_data_adc0;
	uint32_t adc_data_adc1;
	uint8_t adc_channel;
	/* data */
};

struct pid_parameter
{
	float pid_p;
	float pid_i;
	float pid_d;
	float pre_err;
	float last_err;
	float pid_pout;
	float pid_iout;
	float pid_dout;
	int pid_out;
	int pid_out_max;
};
typedef struct pid_parameter* pid_pointer;

//extern unsigned char gImage_bg;
double temp_calcu(unsigned int x1,double y1,unsigned int x2,double y2,double res_val);
double pre_temp(double res_val);
double get_preres(double vadc);

u8 SYS_STATE = SYS_NORMAL;
struct struct_adc_data myadc_data;
struct pid_parameter my_pid;
pid_pointer my_pid_pointer = &my_pid;

u16 temp_set = 250;
double pre_temperature = 0;
uint32_t OUT_PWM = 40;
// 在全局变量区域添加以下变量
uint8_t rainbow_index = 0;  // 流水灯索引
uint32_t rainbow_timer = 0; // 流水灯定时器
uint32_t led_timer;
Color_TypeDef temp1;
Color_TypeDef temp2;
Color_TypeDef temp3;
Color_TypeDef temp4;
Color_TypeDef temp5;
Color_TypeDef temp6;        // 白色状态颜色

double smooth_temperature(double raw_temp);     //温度平滑处理
void update_display(void);                      // 更新显示
void adc_auto_channel(void);                    // 自动选择ADC通道
void color_init(void);                          // 初始化RGB灯
void mypid_init(void);                          // 初始化PID参数
void pid_calculate(void);                       // PID计算
void rgb_change(void);                          // RGB灯颜色渐变
void draw_table_frame(void);                    // 绘制表格框架

int main(void)
{
    u16 PRE_TEMP;
    double vadc = 0;
    double pre_res = 0;
    HAL_Init();                                             /* 初始化HAL库 */
    //sys_stm32_clock_init(RCC_PLL_MUL9);                   /* 设置时钟, 72Mhz */
    sys_stm32_clock_init(RCC_PLL_MUL4);
    delay_init(72);                                         /* 延时初始化 */
    usart_init(115200);                                     /* 初始化串口 */
    led_init();                                             /* 初始化LED */
    gtim_timx_pwm_chy_init(500 - 1, 72 - 1);                /* 1Mhz的计数频率,2Khz的PWM. */
    __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, GTIM_TIMX_PWM_CHY, 500 - 0*5);
    beep_init();
    fan_init();
    key_init();
    seg_init();
    exti_init();
    adc_init();
    
    LCD_Init();                                             // LCD初始化
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);                    // 填充背景色
    
    // 绘制表格框架
    draw_table_frame();
    
    // 显示固定标签文本
    LCD_ShowString(10, 10, "STAT:", CYAN, BLACK, 32, 0);
	LCD_ShowString(10, 53, "SET:", LIGHTBLUE, BLACK, 32, 0);
	LCD_ShowString(10, 96, "TEMP:", GREEN, BLACK, 32, 0);
    
    WS2812B_Init();	
    color_init();
    mypid_init();
    FAN(0);
    WS2812B_FillColor(0, 60, &temp1);
    WS2812B_RefreshPixel();
    
    while (1)
    {
        adc_auto_channel();
        
        if(myadc_data.adc_channel == 0)
        {
            PRE_TEMP = myadc_data.adc_data_adc0;
            vadc = PRE_TEMP*3.3/4096.0;
        }
        else
        {
            PRE_TEMP = myadc_data.adc_data_adc1;
            vadc = PRE_TEMP*3.3/4096.0/21.0;
        }
		
        pre_res = get_preres(vadc);
        
        // 修正电阻值范围检查 - 更合理的范围
        if(pre_res < 100 || pre_res > 5000000) {
            // 不要强制设置默认值，而是允许一定范围内的异常值
            // 只在明显错误时才使用默认值
            if(pre_res < 10 || pre_res > 10000000) {
                pre_res = 92180; // 设置为25°C时的实际阻值
            }
        }
        
        pre_temperature = pre_temp(pre_res);  // 使用新的NTC B值公式计算温度
        
        // 移除过于严格的温度范围检查，允许温度正常变化
        // 只在明显错误时进行修正
        if(pre_temperature < -100 || pre_temperature > 600) {
            pre_temperature = 25; // 只在极端异常时设置默认值
        }
        
        pre_temperature = smooth_temperature(pre_temperature);  // 添加平滑处理
        
		// 在主循环中添加以下代码（放在while(1)循环内合适位置）
		if(pre_temperature <= 40)
		{
			if(HAL_GetTick() - rainbow_timer > 20) // 每20ms更新一次流水灯
			{
				rainbow_timer = HAL_GetTick();
				rainbow_index++;
				if(rainbow_index >= 60) rainbow_index = 0;
			}
		}
        // 使用新的更新显示函数
        update_display();
        
        if(HAL_GetTick() - led_timer > 200) // 每200ms更新一次LED
		{
			led_timer = HAL_GetTick();
			LED0_TOGGLE();
		}
        if(SYS_STATE == SYS_NORMAL)
        {
            __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, GTIM_TIMX_PWM_CHY, 500 - 0*5);
            if(pre_temperature > 40)
            {
                //打开风扇
                FAN(1);
            }
            else
            {
                //关闭风扇
                FAN(0);
            }
        }
        else if(SYS_STATE == SYS_WORKING)
        {
            FAN(0);
            pid_calculate();
            __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, GTIM_TIMX_PWM_CHY, 500 - my_pid_pointer->pid_out*5);
        }
        delay_ms(10);
    }
}

//ADC通道自动切换且读取ADC数据函数
//ADC通道自动切换且读取ADC数据函数
void adc_auto_channel(void)
{
	uint32_t data = 0;
	SEG_ADC0;
	data = adc_get_result_average(ADC_CHANNEL_8,5);
	// 修改判断条件，使用更合理的阈值
	if(data >= 200)  // 原来是100，改为200更合理
	{
		myadc_data.adc_channel = 0;
		myadc_data.adc_data_adc0 = data;
	}
	else
	{
		SEG_ADC1;
		data = adc_get_result_average(ADC_CHANNEL_8,5);
		myadc_data.adc_channel = 1;
		myadc_data.adc_data_adc1 = data;
	}
}

//通过adc的电压值得到当前的电阻值
double get_preres(double vadc)
{
    double pre_res = 0;
    
    // 防止除零错误和无效值
    if(vadc >= 3.29) vadc = 3.29;
    if(vadc <= 0.001) vadc = 0.001;
    
    // 根据分压电路计算NTC电阻值
    // 电路连接: 3.3V -- 130K电阻 -- ADC点 -- NTC热敏电阻 -- GND
    // Vadc = 3.3 * (NTC_R / (NTC_R + 130K))
    // NTC_R = 130K * (Vadc / (3.3 - Vadc))
    pre_res = 130000.0 * vadc / (3.3 - vadc);
    return pre_res;
}

// 使用标准的NTC B值公式计算温度
// 通过输入当前读取的电阻值获取当前的温度 (使用NTC B值公式)
double pre_temp(double res_val)
{
    double temperature = 0;
    double lnR;
    double tempK;
    
    // 防止无效值
    if(res_val <= 0) res_val = 1;
    if(res_val > 10000000) res_val = 10000000; // 防止过大值
    
    // 使用NTC热敏电阻的标准B值公式:
    // 1/T = 1/T0 + ln(R/R0)/B
    // T = 1/(1/T0 + ln(R/R0)/B) - 273.15 (转换为摄氏度)
    
    // 根据实际测量修正参数
    const double ACTUAL_R25 = 92180.0;  // 25°C时实际测量的阻值 92.18K
    const double BETA = 3950.0;         // B值
    const double T25 = 298.15;          // 25°C对应的开尔文温度
    
    lnR = log(res_val / ACTUAL_R25);    // ln(R/R0)
    tempK = 1.0 / (1.0/T25 + lnR/BETA); // 开尔文温度
    temperature = tempK - 273.15;       // 转换为摄氏度
    
    // 限制温度范围，防止异常值
    if(temperature < -50) temperature = -50;
    if(temperature > 500) temperature = 500;
    
    return temperature;
}

//通过输入当前读取的电阻值获取当前的温度
// double pre_temp(double res_val)
// {
// 	double pre_temp = 0;
// 	u8 temp_range;
// 	u8 i = 0;
// 	for(i=0;i<=100;i++)
// 	{
// 		if(res_val >= data_y[i])
// 		{
// 			temp_range = i*3;
// 			break;
// 		}
// 	}
// 	pre_temp = temp_calcu(temp_range-3,data_y[i-1],temp_range,data_y[i],res_val);
// 	return pre_temp;
// }
// //根据当前电阻数字所在的区间构建线性方程，并求对应的温度
// double temp_calcu(unsigned int x1,double y1,unsigned int x2,double y2,double res_val)
// {
// 	double retemp = 0;
// 	double k;
// 	double b;
// 	k = (double)(y2 - y1)/(x2 - x1);
// 	b = (double)y2 - k*(double)x2;
// 	retemp = (res_val-b)/k;
// 	return retemp;
// }
void mypid_init(void)
{
	my_pid_pointer->pre_err = 0;
	my_pid_pointer->last_err = 0;
	my_pid_pointer->pid_pout = 0;
	my_pid_pointer->pid_iout = 0;
	my_pid_pointer->pid_dout = 0;
	my_pid_pointer->pid_out = 0;
	my_pid_pointer->pid_out_max = 100;  // 增加最大输出值，原来是50可能太小
	my_pid_pointer->pid_p = 0.3;        // 增加比例系数，原来是0.1可能太小
	my_pid_pointer->pid_i = 0.01;       // 添加积分系数
	my_pid_pointer->pid_d = 0.05;       // 微分系数可以保持不变
}
void pid_calculate(void)
{
	my_pid_pointer->last_err = my_pid_pointer->pre_err;
	my_pid_pointer->pre_err = temp_set - pre_temperature;
	my_pid_pointer->pid_pout = my_pid_pointer->pre_err * my_pid_pointer->pid_p;
	my_pid_pointer->pid_dout = my_pid_pointer->pid_d * (my_pid_pointer->pre_err - my_pid_pointer->last_err);
	my_pid_pointer->pid_out = my_pid_pointer->pid_pout + my_pid_pointer->pid_dout;
	if(my_pid_pointer->pid_dout > my_pid_pointer->pid_out_max)
		my_pid_pointer->pid_out = my_pid_pointer->pid_out_max;
	else if(my_pid_pointer->pid_out < 0)
		my_pid_pointer->pid_out = 0;
}
void color_init(void){
	temp1.R = 0x7C;
	temp1.G = 0xfC;
	temp1.B = 0x00;  //紫色

	temp2.R = 0x87;
	temp2.G = 0xCE;
	temp2.B = 0xFA;  // 冰蓝色 (Light Sky Blue)

	temp3.R = 0x9A;
	temp3.G = 0xCD;
	temp3.B = 0x32;  // 黄绿色 (Yellow-Green)

	temp4.R = 0xFF;
	temp4.G = 0xFF;
	temp4.B = 0x00;  // 黄色
    
    // 橙色保持不变
    temp5.R = 0xFF;
	temp5.G = 0x61;
	temp5.B = 0x00; // 橙色

    // 红色 (使用原来的temp6位置)
    temp6.R = 0xFF;
	temp6.G = 0x00;
	temp6.B = 0x00; // 红色
}
// 添加新的颜色生成函数
void generate_rainbow_color(uint8_t position, Color_TypeDef *color)
{
    uint8_t hue = position % 60; // 创建颜色流动效果
    
    if(hue < 12) {
        // 红到黄过渡
        color->R = 0xFF;
        color->G = hue * 21;  // 0-252
        color->B = 0x00;
    }
    else if(hue < 24) {
        // 黄到绿过渡
        color->R = 0xFF - ((hue-12) * 21); // 252-0
        color->G = 0xFF;
        color->B = 0x00;
    }
    else if(hue < 36) {
        // 绿到青过渡
        color->R = 0x00;
        color->G = 0xFF;
        color->B = (hue-24) * 21; // 0-252
    }
    else if(hue < 48) {
        // 青到蓝过渡
        color->R = 0x00;
        color->G = 0xFF - ((hue-36) * 21); // 252-0
        color->B = 0xFF;
    }
    else {
        // 蓝到红过渡
        color->R = (hue-48) * 21; // 0-252
        color->G = 0x00;
        color->B = 0xFF;
    }
}

// 修改rgb_change函数为基于状态机的版本
void rgb_change(void)
{
    RGB_State_TypeDef new_state;
    
    // 根据温度和系统状态确定目标状态
    if(SYS_STATE == SYS_NORMAL && pre_temperature <= 40)
    {
        new_state = RGB_STATE_RAINBOW;        // 不加热且温度≤40°C，显示彩虹流水灯
    }
    else if(SYS_STATE == SYS_WORKING && pre_temperature < 40)
    {
        new_state = RGB_STATE_BLUE;           // 加热且温度<40°C，显示冰蓝色
    }
    else if(pre_temperature >= 40 && pre_temperature <= 70)
    {
        new_state = RGB_STATE_YELLOW_GREEN;   // 40-70°C，显示浅黄绿色
    }
    else if(pre_temperature > 70 && pre_temperature <= 100)
    {
        new_state = RGB_STATE_YELLOW;         // 70-100°C，显示黄色
    }
    else if(pre_temperature > 100 && pre_temperature <= 150)
    {
        new_state = RGB_STATE_ORANGE;         // 100-150°C，显示橙色
    }
    else if(pre_temperature > 150 && pre_temperature <= 200)
    {
        new_state = RGB_STATE_RED;            // 150-200°C，显示红色
    }
    else
    {
        new_state = RGB_STATE_PURPLE;         // >200°C，显示紫色
    }
    
    // 检查状态是否发生变化
    if(new_state != rgb_current_state)
    {
        rgb_current_state = new_state;
        rgb_state_changed = 1;
    }
    
    // 根据状态执行相应操作
    switch(rgb_current_state)
    {
        case RGB_STATE_RAINBOW:
            // 彩色流水灯效果 - 只有在SYS_STATE == SYS_NORMAL且温度<=40时才启用
            if(rgb_state_changed || 1) // 总是更新以保持动画效果
            {
                for(int i = 0; i < 60; i++)
                {
                    uint8_t color_position = (i + rainbow_index) % 60;
                    Color_TypeDef color;
                    generate_rainbow_color(color_position, &color);
                    Copy_Color(&PixelBuf[i], &color);
                }
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_BLUE:
            // 冰蓝色状态 - 加热时温度<40°C
            if(rgb_state_changed)
            {
                WS2812B_FillColor(0, 60, &temp2);  // 使用冰蓝色
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_YELLOW_GREEN:
            // 浅黄绿色状态 - 40-70°C
            if(rgb_state_changed)
            {
                WS2812B_FillColor(0, 60, &temp3);  // 使用浅黄绿色
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_YELLOW:
            // 黄色状态 - 70-100°C
            if(rgb_state_changed)
            {
                WS2812B_FillColor(0, 60, &temp4);  // 使用黄色
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_ORANGE:
            // 橙色状态 - 100-150°C
            if(rgb_state_changed)
            {
                WS2812B_FillColor(0, 60, &temp5);  // 使用橙色
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_RED:
            // 红色状态 - 150-200°C
            if(rgb_state_changed)
            {
                WS2812B_FillColor(0, 60, &temp6);  // 使用红色
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        case RGB_STATE_PURPLE:
            // 紫色状态 - >200°C
            if(rgb_state_changed)
            {
                // 创建一个新的Color_TypeDef变量用于紫色
                Color_TypeDef purple_color;
                purple_color.R = 0x80;
                purple_color.G = 0x00;
                purple_color.B = 0x80; // 紫色
                
                WS2812B_FillColor(0, 60, &purple_color);
                WS2812B_RefreshPixel();
                rgb_state_changed = 0;
            }
            break;
            
        default:
            break;
    }
}

// 新增函数：绘制指定宽度的水平线（加粗线条）
void draw_thick_hline(u16 x1, u16 y, u16 x2, u16 color, u8 thickness)
{
    u8 i;
    for(i = 0; i < thickness; i++)
    {
        LCD_DrawLine(x1, y + i, x2, y + i, color);
    }
}

// 新增函数：绘制指定宽度的垂直线（加粗线条）
void draw_thick_vline(u16 x, u16 y1, u16 y2, u16 color, u8 thickness)
{
    u8 i;
    for(i = 0; i < thickness; i++)
    {
        LCD_DrawLine(x + i, y1, x + i, y2, color);
    }
}

// 新增函数：绘制指定宽度的矩形框（加粗线条）
void draw_thick_rectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color, u8 thickness)
{
    u8 i;
    for(i = 0; i < thickness; i++)
    {
        LCD_DrawRectangle(x1 + i, y1 + i, x2 - i, y2 - i, color);
    }
}

// 修改函数：绘制带边框的表格区域（加粗线条，3像素宽，只显示三行）
void draw_table_frame(void)
{
    // 绘制外边框 (3像素宽)
    draw_thick_rectangle(2, 2, LCD_W-2, LCD_H-2, WHITE, 3);
    
    // 绘制水平分割线 (3像素宽，将屏幕分为三行)
    draw_thick_hline(2, 45, LCD_W-2, WHITE, 3);    // 第一行分割线（状态）
    draw_thick_hline(2, 90, LCD_W-2, WHITE, 3);   // 第二行分割线（设定温度）
    
    // 绘制竖直分割线 (3像素宽)
    draw_thick_vline(90, 2, LCD_H-2, WHITE, 3); // 标签和数值之间的分割线
}

// 更新显示函数，包括界面框架和数据显示（每行使用不同颜色）
void update_display(void)
{
    // 更新状态显示（第一行，使用红色）
    if(SYS_STATE == SYS_NORMAL)
    {
        LCD_ShowString(100, 10, "OFF", CYAN, BLACK, 32, 0);
    }
    else if(SYS_STATE == SYS_WORKING)
    {
        LCD_ShowString(100, 10, "ON ", CYAN, BLACK, 32, 0);
    }
    
    // 更新设定温度显示（第二行，使用浅蓝色）
    LCD_ShowIntNum(100, 53, temp_set, 3, LIGHTBLUE, BLACK, 32);
    LCD_ShowString(100 + 3*16, 53, " C", LIGHTBLUE, BLACK, 32, 0);  // 添加温度单位（空格+C）
    
    // 更新当前温度显示（第三行，使用绿色）
    //LCD_ShowFloatNum1(100, 96, pre_temperature, 5, GREEN, BLACK, 32);
    LCD_ShowIntNum(100, 96, (int)pre_temperature, 3, GREEN, BLACK, 32);
    LCD_ShowString(100 + 3*16, 96, " C", GREEN, BLACK, 32, 0);  // 添加温度单位（空格+C）
    
    // 更新RGB灯显示
    rgb_change();
}

/**
 * @brief  对温度值进行平滑处理，使用移动平均算法
 * @param  raw_temp: 原始温度值
 * @retval 平滑后的温度值
 */
double smooth_temperature(double raw_temp)
{
    static double sum = 0;
    double smoothed_temp;
    
    // 将新值添加到缓冲区，并更新索引
    sum -= temp_buffer[temp_buffer_index];  // 从总和中减去即将被替换的旧值
    temp_buffer[temp_buffer_index] = raw_temp;  // 存储新值
    sum += raw_temp;  // 将新值加入总和
    
    // 更新索引
    temp_buffer_index++;
    if(temp_buffer_index >= TEMP_SMOOTHING_SIZE)
    {
        temp_buffer_index = 0;
        temp_buffer_full = 1;  // 标记缓冲区已满
    }
    
    // 计算平均值
    if(temp_buffer_full)
    {
        // 缓冲区已满，使用完整窗口大小计算平均值
        smoothed_temp = sum / TEMP_SMOOTHING_SIZE;
    }
    else
    {
        // 缓冲区未满，使用当前实际存储的数据数量计算平均值
        smoothed_temp = sum / (temp_buffer_index == 0 ? TEMP_SMOOTHING_SIZE : temp_buffer_index);
    }
    
    return smoothed_temp;
}


