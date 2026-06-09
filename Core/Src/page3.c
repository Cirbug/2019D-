#include "main.h"
#include "page3.h"
#include "adc.h"
#include "lcd.h"
#include "AD9954.h"

extern uint32_t page3_freq;
extern uint16_t page3_adc3_ac;
extern uint16_t page3_adc3_dc;

// ============================================================
//  界面3：DDS 手动频率控制 (320x240)
//  trigger1/2/3 全部低电平
//  Key_UP: 频率+10kHz (1k~500k, 回绕到1k)
// ============================================================
void Page3_Init(void)
{
    LCD_Clear(WHITE);

    POINT_COLOR = RED;
    LCD_ShowString(5, 2, 200, 28, 24, (uint8_t *)"DDS Control");

    POINT_COLOR = BLACK;
    LCD_ShowString(5, 40, 80, 22, 16, (uint8_t *)"Freq:");

    POINT_COLOR = BLUE;
    if(page3_freq >= 1000)
        LCD_ShowNum(60, 40, page3_freq / 1000, 5, 16);
    else
        LCD_ShowString(60, 40, 60, 22, 16, (uint8_t *)"---");
    POINT_COLOR = BLACK;
    LCD_ShowString(140, 40, 40, 22, 16, (uint8_t *)"kHz");

    POINT_COLOR = GRAY;
    LCD_ShowString(5, 75, 100, 22, 16, (uint8_t *)"ADC3ac:");
    LCD_ShowString(5, 105, 100, 22, 16, (uint8_t *)"ADC3dc:");

    POINT_COLOR = BLUE;
    LCD_ShowString(80, 75, 80, 22, 16, (uint8_t *)"---");
    LCD_ShowString(80, 105, 80, 22, 16, (uint8_t *)"---");

    POINT_COLOR = RED;
    LCD_ShowString(5, 218, 310, 20, 16, (uint8_t *)"Key_UP:+10k  Key0:Change");

    POINT_COLOR = BLACK;
    LCD_ShowString(5, 150, 300, 22, 16, (uint8_t *)"1kHz ~ 500kHz, step=10kHz");

    page3_freq = 1000;
    AD9954_Set_Fre(1000.0);
    AD9954_Set_Amp(1200);
}

// ============================================================
//  界面3：更新 ADC3 双通道显示
// ============================================================
void Page3_Update(void)
{
    static uint16_t last_disp_ac = 0xFFFF;
    static uint16_t last_disp_dc = 0xFFFF;
    #define P3_DISP_THRESHOLD 10

    {
        uint32_t sum_ac = 0, sum_dc = 0;
        #define P3_AVG_CNT 8
        HAL_ADC_Start(&hadc3);
        for(int i = 0; i < P3_AVG_CNT; i++)
        {
            HAL_ADC_PollForConversion(&hadc3, 10);
            sum_ac += (uint16_t)HAL_ADC_GetValue(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 10);
            sum_dc += (uint16_t)HAL_ADC_GetValue(&hadc3);
        }
        HAL_ADC_Stop(&hadc3);
        page3_adc3_ac = (uint16_t)(sum_ac / P3_AVG_CNT);
        page3_adc3_dc = (uint16_t)(sum_dc / P3_AVG_CNT);
        #undef P3_AVG_CNT
    }

    {
        int diff = (int)page3_adc3_ac - (int)last_disp_ac;
        if(diff < 0) diff = -diff;
        if(diff > P3_DISP_THRESHOLD)
        {
            LCD_Fill(80, 75, 160, 97, WHITE);
            POINT_COLOR = BLUE;
            LCD_ShowNum(80, 75, page3_adc3_ac, 4, 16);
            last_disp_ac = page3_adc3_ac;
        }
    }

    {
        int diff = (int)page3_adc3_dc - (int)last_disp_dc;
        if(diff < 0) diff = -diff;
        if(diff > P3_DISP_THRESHOLD)
        {
            LCD_Fill(80, 105, 160, 127, WHITE);
            POINT_COLOR = BLUE;
            LCD_ShowNum(80, 105, page3_adc3_dc, 4, 16);
            last_disp_dc = page3_adc3_dc;
        }
    }

    #undef P3_DISP_THRESHOLD
}