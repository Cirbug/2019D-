#include "main.h"
#include "page2.h"
#include "adc.h"
#include "lcd.h"
#include "AD9954.h"
#include <stdio.h>

#define FAULT_DDS_AMP  1200

extern volatile uint8_t fault_running;
extern volatile uint8_t fault_status[4];

// 页面2跨步骤变量
static uint16_t p2_u_load = 0;
static uint16_t p2_adc1_low = 0;
static uint16_t p2_adc2_low = 0;
static uint16_t p2_adc3_ac_low = 0;
static uint16_t p2_adc3_dc_low = 0;
static uint16_t p2_adc1_high = 0;
static uint16_t p2_adc2_high = 0;
static uint16_t p2_adc3_ac_high = 0;
static uint16_t p2_adc3_dc_high = 0;

static const char *fault_str[3] = {"OK","SHORT","OPEN "};

void Page2_Init(void)
{
    LCD_Clear(WHITE);

    POINT_COLOR = RED;
    LCD_ShowString(5, 2, 200, 28, 24, (uint8_t *)"Gu Zhang Jian Ce");

    POINT_COLOR = BLUE;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");

    POINT_COLOR = GRAY;
    LCD_ShowString(5, 35, 140, 22, 16, (uint8_t *)"LOW CH:");
    LCD_ShowString(5, 58, 140, 22, 16, (uint8_t *)"ADC1:");
    LCD_ShowString(5, 81, 140, 22, 16, (uint8_t *)"ADC2:");
    LCD_ShowString(5, 104, 140, 22, 16, (uint8_t *)"ADC3ac:");
    LCD_ShowString(5, 127, 140, 22, 16, (uint8_t *)"ADC3dc:");

    POINT_COLOR = BLACK;
    LCD_ShowString(170, 35, 140, 22, 16, (uint8_t *)"HIGH CH:");
    LCD_ShowString(170, 58, 140, 22, 16, (uint8_t *)"ADC1:");
    LCD_ShowString(170, 81, 140, 22, 16, (uint8_t *)"ADC2:");
    LCD_ShowString(170, 104, 140, 22, 16, (uint8_t *)"ADC3ac:");
    LCD_ShowString(170, 127, 140, 22, 16, (uint8_t *)"ADC3dc:");

    POINT_COLOR = BLACK;
    LCD_ShowString(5, 155, 30, 22, 16, (uint8_t *)"R1:");
    LCD_ShowString(85, 155, 30, 22, 16, (uint8_t *)"R2:");
    LCD_ShowString(165, 155, 30, 22, 16, (uint8_t *)"R3:");
    LCD_ShowString(245, 155, 30, 22, 16, (uint8_t *)"R4:");
    for(int i = 0; i < 4; i++)
    {
        uint16_t x = 25 + (i * 80);
        POINT_COLOR = BLUE;
        LCD_ShowString(x, 175, 72, 22, 16, (uint8_t *)"---");
    }

    POINT_COLOR = RED;
    LCD_ShowString(5, 218, 310, 20, 16, (uint8_t *)"Key0:Change  Key_UP:Start/Stop");
}

void Page2_StartMeasure(void)
{
    fault_running = !fault_running;
    
    if(fault_running)
    {
        POINT_COLOR = GREEN;
        LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"RUN ");
        AD9954_Set_Fre(1000.0);
        AD9954_Set_Amp(FAULT_DDS_AMP);
        HAL_Delay(10);
    }
    else
    {
        POINT_COLOR = BLUE;
        LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");
        AD9954_Set_Fre(1000.0);
        AD9954_Set_Amp(1200);
    }
}

void Page2_Update(void)
{
    static uint8_t p2_mstep = 0;

    if(p2_mstep == 0)
    {
        SET_TRIGGER2(GPIO_PIN_RESET);
        HAL_Delay(200);

        uint32_t sum1 = 0, sum2 = 0;
        for(int i = 0; i < 8; i++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_Delay(1);
            sum1 += HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Start(&hadc2);
            HAL_Delay(1);
            sum2 += HAL_ADC_GetValue(&hadc2);
        }
        p2_adc1_low = (uint16_t)(sum1 / 8);
        p2_adc2_low = (uint16_t)(sum2 / 8);

        {
            uint32_t sum_ac = 0, sum_dc = 0;
            #define P2_SAMPLE_CNT 12
            HAL_ADC_Start(&hadc3);
            for(int i = 0; i < P2_SAMPLE_CNT; i++)
            {
                HAL_ADC_PollForConversion(&hadc3, 10);
                sum_ac += (uint16_t)HAL_ADC_GetValue(&hadc3);
                HAL_ADC_PollForConversion(&hadc3, 10);
                sum_dc += (uint16_t)HAL_ADC_GetValue(&hadc3);
            }
            HAL_ADC_Stop(&hadc3);
            p2_adc3_ac_low = (uint16_t)(sum_ac / P2_SAMPLE_CNT);
            p2_adc3_dc_low = (uint16_t)(sum_dc / P2_SAMPLE_CNT);
            #undef P2_SAMPLE_CNT
        }
        p2_mstep = 1;
    }
    else if(p2_mstep == 1)
    {
        SET_TRIGGER2(GPIO_PIN_SET);
        HAL_Delay(50);

        uint32_t sum1 = 0, sum2 = 0;
        for(int i = 0; i < 8; i++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_Delay(1);
            sum1 += HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Start(&hadc2);
            HAL_Delay(1);
            sum2 += HAL_ADC_GetValue(&hadc2);
        }
        p2_adc1_high = (uint16_t)(sum1 / 8);
        p2_adc2_high = (uint16_t)(sum2 / 8);

        {
            uint32_t sum_ac = 0, sum_dc = 0;
            #define P2_SAMPLE_CNT 12
            HAL_ADC_Start(&hadc3);
            for(int i = 0; i < P2_SAMPLE_CNT; i++)
            {
                HAL_ADC_PollForConversion(&hadc3, 10);
                sum_ac += (uint16_t)HAL_ADC_GetValue(&hadc3);
                HAL_ADC_PollForConversion(&hadc3, 10);
                sum_dc += (uint16_t)HAL_ADC_GetValue(&hadc3);
            }
            HAL_ADC_Stop(&hadc3);
            p2_adc3_ac_high = (uint16_t)(sum_ac / P2_SAMPLE_CNT);
            p2_adc3_dc_high = (uint16_t)(sum_dc / P2_SAMPLE_CNT);
            #undef P2_SAMPLE_CNT
        }
        p2_mstep = 2;
    }
    else
    {
        SET_TRIGGER2(GPIO_PIN_RESET);

        #define P2_DISP_THRESHOLD 20
        static uint16_t p2_last_disp[8] = {0,0,0,0,0,0,0,0};
        {
            uint16_t newvals[4] = {p2_adc1_low, p2_adc2_low, p2_adc3_ac_low, p2_adc3_dc_low};
            static const uint16_t xs[4] = {55, 55, 55, 55};
            static const uint16_t ys[4] = {58, 81, 104, 127};
            for(int i = 0; i < 4; i++)
            {
                int diff = (int)newvals[i] - (int)p2_last_disp[i];
                if(diff < 0) diff = -diff;
                if(diff > P2_DISP_THRESHOLD)
                {
                    POINT_COLOR = GRAY;
                    LCD_Fill(xs[i], ys[i], 160, ys[i]+22, WHITE);
                    LCD_ShowNum(xs[i], ys[i], newvals[i], 4, 16);
                    p2_last_disp[i] = newvals[i];
                }
            }
        }
        {
            uint16_t newvals[4] = {p2_adc1_high, p2_adc2_high, p2_adc3_ac_high, p2_adc3_dc_high};
            static const uint16_t xs[4] = {225, 225, 225, 225};
            static const uint16_t ys[4] = {58, 81, 104, 127};
            for(int i = 0; i < 4; i++)
            {
                int diff = (int)newvals[i] - (int)p2_last_disp[4+i];
                if(diff < 0) diff = -diff;
                if(diff > P2_DISP_THRESHOLD)
                {
                    POINT_COLOR = BLACK;
                    LCD_Fill(xs[i], ys[i], 315, ys[i]+22, WHITE);
                    LCD_ShowNum(xs[i], ys[i], newvals[i], 4, 16);
                    p2_last_disp[4+i] = newvals[i];
                }
            }
        }
        #undef P2_DISP_THRESHOLD

        if(p2_adc3_ac_high > 2000)
            fault_status[0] = 1;
        else if(p2_adc3_ac_high >2000)
            fault_status[0] = 2;
        else
            fault_status[0] = 0;

        if((p2_adc2_low<20) &&(380<p2_adc3_dc_low)&&( p2_adc3_dc_low< 420))
            fault_status[1] = 1;
        else if((400<p2_adc2_low)&& (p2_adc2_low<450) &&( p2_adc3_ac_low< 20)&&(120<p2_adc3_dc_low)&&(p2_adc3_dc_low < 160))
            fault_status[1] = 2;
        else
            fault_status[1] = 0;

        if((180<p2_adc2_low)&& (p2_adc2_low<220) &&(380<p2_adc3_dc_low)&&( p2_adc3_dc_low< 420))
            fault_status[2] = 1;
        else if((10<p2_adc2_low)&& (p2_adc2_low<40) &&(p2_adc3_dc_low< 20)&&(p2_adc3_ac_low< 20))
            fault_status[2] = 2;
        else
            fault_status[2] = 0;

        if((p2_adc2_low<5) &&( p2_adc3_dc_low< 20)&&( p2_adc3_ac_low< 20))
            fault_status[3] = 1;
        else if( p2_adc3_ac_low< 10)
            fault_status[3] = 2;
        else
            fault_status[3] = 0;

        static const uint16_t status_color[3] = {GREEN, RED, BLUE};
        for(int i = 0; i < 4; i++)
        {
            uint16_t x = 25 + (i * 80);
            LCD_Fill(x, 175, x + 72, 197, WHITE);
            POINT_COLOR = status_color[fault_status[i]];
            LCD_ShowString(x, 175, 72, 22, 16, (uint8_t *)fault_str[fault_status[i]]);
        }

        p2_mstep = 0;
    }
}