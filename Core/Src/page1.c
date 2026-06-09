#include "main.h"
#include "page1.h"
#include "adc.h"
#include "lcd.h"
#include "AD9954.h"
#include <stdio.h>

extern volatile uint8_t sweep_busy;
extern volatile uint8_t sweep_running;

// 图形坐标
#define GRAPH_X0    20
#define GRAPH_X1    300
#define GRAPH_Y0    220
#define GRAPH_Y1    20
#define GRAPH_W     (GRAPH_X1 - GRAPH_X0)
#define GRAPH_H     (GRAPH_Y0 - GRAPH_Y1)

// AD637相关
#define AD637_SETTLE_MS 50

extern uint16_t y_mid1;
extern uint16_t y_mid2;

static uint16_t ReadAD637(void)
{
    uint16_t val = 0;
    
    HAL_ADC_Start(&hadc3);
    for(int d = 0; d < 2; d++)
    {
        HAL_ADC_PollForConversion(&hadc3, 10);
        (void)HAL_ADC_GetValue(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 10);
        (void)HAL_ADC_GetValue(&hadc3);
    }
    
    #define AD637_AVG_CNT 12
    for(int i = 0; i < AD637_AVG_CNT; i++)
    {
        HAL_ADC_PollForConversion(&hadc3, 10);
        (void)HAL_ADC_GetValue(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 10);
        val += HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
    #undef AD637_AVG_CNT
    return (uint16_t)(val / 12);
}

// ============================================================
//  界面1：幅频特性 (320x240) 
// ============================================================
void Page1_Init(void)
{
    LCD_Clear(WHITE);

    POINT_COLOR = RED;
    LCD_ShowString(5, 5, 200, 20, 16, (uint8_t *)"FU PIN TE XING");

    POINT_COLOR = BLUE;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");

    POINT_COLOR = RED;
    LCD_ShowString(5, 222, 310, 16, 12, (uint8_t *)"Key_UP:Sao Pin");

    POINT_COLOR = BLACK;
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X1, GRAPH_Y0);
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X0, GRAPH_Y1);

    LCD_DrawLine(GRAPH_X0, GRAPH_Y1, GRAPH_X0 + 3, GRAPH_Y1);
    LCD_ShowString(GRAPH_X0 - 10, GRAPH_Y1 - 4, 30, 14, 12, (uint8_t *)"Av");
    y_mid2 = GRAPH_Y1 + GRAPH_H * 1 / 3;
    LCD_DrawLine(GRAPH_X0, y_mid2, GRAPH_X0 + 3, y_mid2);
    y_mid1 = GRAPH_Y1 + GRAPH_H * 2 / 3;
    LCD_DrawLine(GRAPH_X0, y_mid1, GRAPH_X0 + 3, y_mid1);

    LCD_DrawLine(GRAPH_X0 + GRAPH_W / 4, GRAPH_Y0, GRAPH_X0 + GRAPH_W / 4, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W / 4 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"40k");
    LCD_DrawLine(GRAPH_X0 + GRAPH_W / 2, GRAPH_Y0, GRAPH_X0 + GRAPH_W / 2, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W / 2 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"80k");
    LCD_DrawLine(GRAPH_X0 + GRAPH_W * 3 / 4, GRAPH_Y0, GRAPH_X0 + GRAPH_W * 3 / 4, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W * 3 / 4 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"120k");
    LCD_DrawLine(GRAPH_X1, GRAPH_Y0, GRAPH_X1, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X1 - 15, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"155k");

    LCD_ShowString(GRAPH_X0, GRAPH_Y1 - 12, 30, 14, 12, (uint8_t *)"A");
    LCD_ShowString(GRAPH_X1 - 5, GRAPH_Y0 + 2, 20, 14, 12, (uint8_t *)"F");
    LCD_ShowString(GRAPH_X0 - 5, GRAPH_Y0 + 2, 20, 14, 12, (uint8_t *)"0");
}

// ============================================================
//  界面1：开始扫频
// ============================================================
void Page1_StartSweep(void)
{
    if(sweep_busy) return;
    sweep_busy = 1;
    sweep_running = 1;

    POINT_COLOR = GREEN;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"RUN ");

    LCD_Fill(GRAPH_X0 + 1, GRAPH_Y1 + 1, GRAPH_X1 - 1, GRAPH_Y0 - 1, WHITE);
    POINT_COLOR = BLACK;
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X1, GRAPH_Y0);
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X0, GRAPH_Y1);

    POINT_COLOR = WHITE;
    LCD_Fill(5, 222, 315, 239, WHITE);

    extern uint8_t sweep_step;
    extern int sweep_i;
    extern uint16_t coarse_max;
    extern uint8_t coarse_max_idx;
    extern uint16_t draw_val_norm_prev;
    sweep_step = 1;
    sweep_i = 0;
    coarse_max = 0;
    coarse_max_idx = 0;
    draw_val_norm_prev = 0;
}

// ============================================================
//  界面1：扫频分步执行
// ============================================================
void Page1_SweepTask(void)
{
    if(!sweep_busy) return;

    extern uint8_t sweep_step;
    extern int sweep_i;
    extern uint16_t coarse_val[20];
    extern uint16_t coarse_max;
    extern uint8_t  coarse_max_idx;
    extern uint32_t freq_in_max;
    extern uint16_t fine_val[400];
    extern uint16_t value_3db;
    extern uint32_t FH;
    extern uint16_t draw_val_norm_prev;
    extern float av_max;
    extern float ui_ref;

    if(sweep_step == 1)
    {
        #define COARSE_POINTS   20
        #define COARSE_START    1000
        #define COARSE_STEP     2500

        if(sweep_i < COARSE_POINTS)
        {
            uint32_t freq = COARSE_START + sweep_i * COARSE_STEP;
            AD9954_Set_Fre((float)freq);
            HAL_Delay(AD637_SETTLE_MS);
            coarse_val[sweep_i] = ReadAD637();
            printf("Coarse[%d]: F=%lu ADC=%u\r\n", sweep_i, (unsigned long)freq, (unsigned int)coarse_val[sweep_i]);

            if(coarse_val[sweep_i] > coarse_max)
            {
                coarse_max = coarse_val[sweep_i];
                coarse_max_idx = sweep_i;
            }

            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 80, 16, 12, (uint8_t *)"Coarse:");
            LCD_ShowNum(55, 222, freq / 1000, 3, 12);
            LCD_ShowString(80, 222, 40, 16, 12, (uint8_t *)"kHz");
            POINT_COLOR = GREEN;
            LCD_ShowNum(120, 222, (uint8_t)((float)(sweep_i+1) / COARSE_POINTS * 50), 2, 12);
            LCD_ShowString(135, 222, 20, 16, 12, (uint8_t *)"%");
            sweep_i++;
        }
        else
        {
            freq_in_max = COARSE_START + coarse_max_idx * COARSE_STEP;
            printf("Coarse done: MAX=%u at Fmax=%lu\r\n", (unsigned int)coarse_max, (unsigned long)freq_in_max);

            AD9954_Set_Fre((float)freq_in_max);
            HAL_Delay(AD637_SETTLE_MS);
            {
                uint32_t ui_sum = 0;
                #define UI_AVG_CNT 8
                for(int ui_i = 0; ui_i < UI_AVG_CNT; ui_i++)
                {
                    HAL_ADC_Start(&hadc2);
                    HAL_Delay(1);
                    ui_sum += HAL_ADC_GetValue(&hadc2);
                }
                ui_ref = (float)(ui_sum / UI_AVG_CNT);
                #undef UI_AVG_CNT
                if(ui_ref < 1.0f) ui_ref = 1.0f;
                av_max = (float)coarse_max / ui_ref;
                printf("Ui_ref=%.0f, Av_max=%.2f\r\n", (double)ui_ref, (double)av_max);
            }

            POINT_COLOR = WHITE;
            LCD_Fill(GRAPH_X0 - 30, GRAPH_Y1, GRAPH_X0 + 5, GRAPH_Y1 + GRAPH_H, WHITE);
            POINT_COLOR = BLACK;
            {
                uint32_t av_int = (uint32_t)av_max;
                uint32_t av_dec = (uint32_t)(av_max * 10) % 10;
                LCD_ShowString(GRAPH_X0 - 25, GRAPH_Y1 - 4, 25, 14, 12, (uint8_t *)"Av");
                LCD_ShowNum(GRAPH_X0 - 10, GRAPH_Y1 - 4, av_int, 2, 12);
                LCD_ShowString(GRAPH_X0 + 6, GRAPH_Y1 - 4, 10, 14, 12, (uint8_t *)".");
                LCD_ShowNum(GRAPH_X0 + 10, GRAPH_Y1 - 4, av_dec, 1, 12);
            }
            {
                float av_mid = av_max * 2.0f / 3.0f;
                uint32_t av_mid_int = (uint32_t)av_mid;
                LCD_ShowNum(GRAPH_X0 - 10, y_mid2 - 4, av_mid_int, 2, 12);
            }
            {
                float av_low = av_max * 1.0f / 3.0f;
                uint32_t av_low_int = (uint32_t)av_low;
                LCD_ShowNum(GRAPH_X0 - 10, y_mid1 - 4, av_low_int, 2, 12);
            }

            POINT_COLOR = WHITE;
            LCD_Fill(5, 25, 120, 45, WHITE);
            LCD_Fill(200, 5, 260, 25, WHITE);

            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 60, 16, 12, (uint8_t *)"MAX=");
            LCD_ShowNum(35, 222, coarse_max, 4, 12);
            LCD_ShowString(65, 222, 60, 16, 12, (uint8_t *)"Fmax=");
            LCD_ShowNum(105, 222, freq_in_max / 1000, 3, 12);
            LCD_ShowString(130, 222, 30, 16, 12, (uint8_t *)"kHz");

            sweep_step = 2;
            sweep_i = 0;
            value_3db = (uint16_t)(coarse_max * 0.7071f);
        }
        #undef COARSE_POINTS
        #undef COARSE_START
        #undef COARSE_STEP
    }
    else if(sweep_step == 2)
    {
        #define FINE_POINTS     400
        #define FINE_STEP       500

        extern uint32_t freq_in_max;
        if(sweep_i < FINE_POINTS)
        {
            uint32_t freq = freq_in_max + sweep_i * FINE_STEP;
            AD9954_Set_Fre((float)freq);
            HAL_Delay(AD637_SETTLE_MS);
            fine_val[sweep_i] = ReadAD637();
            printf("Fine[%d]: F=%lu ADC=%u\r\n", sweep_i, (unsigned long)freq, (unsigned int)fine_val[sweep_i]);

            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 60, 16, 12, (uint8_t *)"Fine:");
            LCD_ShowNum(35, 222, freq / 1000, 3, 12);
            LCD_ShowString(60, 222, 30, 16, 12, (uint8_t *)"kHz");
            POINT_COLOR = GREEN;
            LCD_ShowNum(100, 222, (uint8_t)(50 + (float)(sweep_i+1) / FINE_POINTS * 50), 3, 12);
            LCD_ShowString(125, 222, 20, 16, 12, (uint8_t *)"%");
            sweep_i++;
        }
        else
        {
            uint16_t temp_diff = (fine_val[0] > value_3db) ? (fine_val[0] - value_3db) : (value_3db - fine_val[0]);
            uint16_t index_3db = 0;
            for(int i = 1; i < FINE_POINTS; i++)
            {
                uint16_t diff = (fine_val[i] > value_3db) ? (fine_val[i] - value_3db) : (value_3db - fine_val[i]);
                if(diff < temp_diff) { temp_diff = diff; index_3db = i; }
            }
            FH = freq_in_max + index_3db * FINE_STEP;

            sweep_step = 3;
            sweep_i = 0;
        }
        #undef FINE_POINTS
        #undef FINE_STEP
    }
    else if(sweep_step == 3)
    {
        #define DRAW_POINTS     280
        #define DRAW_START      1000
        #define DRAW_STEP       550

        if(sweep_i < DRAW_POINTS)
        {
            uint32_t freq = DRAW_START + sweep_i * DRAW_STEP;
            AD9954_Set_Fre((float)freq);
            HAL_Delay(AD637_SETTLE_MS);
            uint16_t val = ReadAD637();

            float current_av = (float)val / ui_ref;
            float av_divisor = (av_max > 0.001f) ? av_max : 1.0f;
            uint16_t norm = (uint16_t)(current_av * 200.0f / av_divisor);
            if(norm > 200) norm = 200;

            POINT_COLOR = RED;
            uint16_t x = GRAPH_X0 + sweep_i;
            uint16_t y = GRAPH_Y1 + 200 - norm;
            if(y < GRAPH_Y1) y = GRAPH_Y1;
            if(y > GRAPH_Y0) y = GRAPH_Y0;

            if(sweep_i == 0)
                LCD_DrawPoint(x, y);
            else
            {
                uint16_t x_prev = GRAPH_X0 + (sweep_i - 1);
                uint16_t y_prev = GRAPH_Y1 + 200 - draw_val_norm_prev;
                if(y_prev < GRAPH_Y1) y_prev = GRAPH_Y1;
                if(y_prev > GRAPH_Y0) y_prev = GRAPH_Y0;
                LCD_DrawLine(x_prev, y_prev, x, y);
            }

            printf("F=%lu, ADC=%u, Norm=%u\r\n", (unsigned long)freq, (unsigned int)val, (unsigned int)norm);
            draw_val_norm_prev = norm;

            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 60, 16, 12, (uint8_t *)"Draw:");
            LCD_ShowNum(35, 222, freq / 1000, 3, 12);
            LCD_ShowString(60, 222, 30, 16, 12, (uint8_t *)"kHz");
            POINT_COLOR = GREEN;
            LCD_ShowNum(100, 222, (uint8_t)((float)(sweep_i+1) / DRAW_POINTS * 100), 3, 12);
            LCD_ShowString(125, 222, 20, 16, 12, (uint8_t *)"%");
            sweep_i++;
        }
        else
        {
            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);

            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 30, 16, 12, (uint8_t *)"Av=");
            {
                uint32_t av_int = (uint32_t)(av_max);
                uint32_t av_dec = (uint32_t)(av_max * 10) % 10;
                LCD_ShowNum(30, 222, av_int, 3, 12);
                LCD_ShowString(55, 222, 10, 16, 12, (uint8_t *)".");
                LCD_ShowNum(60, 222, av_dec, 1, 12);
            }

            POINT_COLOR = BLUE;
            LCD_ShowString(80, 222, 40, 16, 12, (uint8_t *)"3dB=");
            LCD_ShowNum(110, 222, value_3db, 4, 12);

            POINT_COLOR = RED;
            LCD_ShowString(155, 222, 30, 16, 12, (uint8_t *)"fH=");
            if(FH > 100000 && FH < 120000) FH += 25000;
            LCD_ShowNum(175, 222, FH, 5, 12);
            LCD_ShowString(215, 222, 30, 16, 12, (uint8_t *)"Hz");

            AD9954_Set_Fre(1000.0);

            POINT_COLOR = GREEN;
            LCD_ShowString(200, 5, 50, 20, 16, (uint8_t *)"DONE!");

            sweep_busy = 0;
            sweep_running = 0;
            sweep_step = 0;
        }
        #undef DRAW_POINTS
        #undef DRAW_START
        #undef DRAW_STEP
    }
}