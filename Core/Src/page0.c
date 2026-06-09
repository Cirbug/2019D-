#include "main.h"
#include "page0.h"
#include "lcd.h"

extern volatile uint8_t measure_running;
extern uint16_t adc1_val;
extern uint16_t adc2_val;
extern uint16_t adc3_val1;
extern uint16_t adc3_val2;
extern uint16_t adc3_valDC;
extern float input_resistance;
extern float output_resistance;
extern float amplification;

// 辅助函数：若新值与旧值差值>阈值则更新（用于显示防抖）
#define UPDATE_IF_CHANGED(newval, oldvar, threshold, refresh_code)   \
    do {                                                             \
        int _diff = (int)(newval) - (int)(oldvar);                   \
        if(_diff < 0) _diff = -_diff;                                \
        if(_diff > (threshold)) {                                    \
            do { refresh_code } while(0);                             \
            oldvar = (newval);                                        \
        }                                                             \
    } while(0)

// ============================================================
//  界面0：参数测量 (320x240)
// ============================================================
void Page0_Init(void)
{
    LCD_Clear(WHITE);

    // --- 顶部标题栏 24字体占28px ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 2, 200, 28, 24, (uint8_t *)"Ce Liang ");

    // --- 状态 ---
    POINT_COLOR = BLUE;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");

    // --- 参数显示区 (16字体, 每行22px) ---
    POINT_COLOR = BLACK;
    // 输入电阻
    LCD_ShowString(5, 35, 160, 22, 16, (uint8_t *)"Rin:");
    POINT_COLOR = BLUE;
    LCD_ShowString(55, 35, 120, 22, 16, (uint8_t *)"---");
    POINT_COLOR = BLACK;
    LCD_ShowString(130, 35, 40, 22, 16, (uint8_t *)" ");

    // 输出电阻
    LCD_ShowString(5, 58, 160, 22, 16, (uint8_t *)"Rout:");
    POINT_COLOR = BLUE;
    LCD_ShowString(55, 58, 120, 22, 16, (uint8_t *)"---");
    POINT_COLOR = BLACK;
    LCD_ShowString(130, 58, 40, 22, 16, (uint8_t *)" ");

    // 放大倍数
    LCD_ShowString(5, 81, 160, 22, 16, (uint8_t *)"Av:");
    POINT_COLOR = BLUE;
    LCD_ShowString(55, 81, 120, 22, 16, (uint8_t *)"---");
    POINT_COLOR = BLACK;
    LCD_ShowString(130, 81, 40, 22, 16, (uint8_t *)"dB");

    // --- ADC原始值 (右侧) ---
    POINT_COLOR = GRAY;
    LCD_ShowString(170, 35, 140, 22, 16, (uint8_t *)"ADC1(Us):");
    LCD_ShowString(170, 58, 140, 22, 16, (uint8_t *)"ADC2(Ui):");
    LCD_ShowString(170, 81, 140, 22, 16, (uint8_t *)"ADC3(load):");
	LCD_ShowString(170, 104, 140, 22, 16, (uint8_t *)"ADC3(open):");
	LCD_ShowString(170, 127, 140, 22, 16, (uint8_t *)"ADC3(DC):");

    // --- 底部按键提示 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 218, 310, 20, 16, (uint8_t *)"Key0:Change  Key_UP:Start/Stop");
}

// ============================================================
//  界面0：更新测量数据（仅变化>20时才刷新）
// ============================================================
void Page0_Update(void)
{
    // 保存上次显示的各ADC值
    static uint16_t last_disp_adc1 = 0xFFFF;
    static uint16_t last_disp_adc2 = 0xFFFF;
    static uint16_t last_disp_adc3_1 = 0xFFFF;
    static uint16_t last_disp_adc3_2 = 0xFFFF;
    static uint16_t last_disp_adc3_dc = 0xFFFF;

    #define CHANGE_THRESHOLD 20

    // === ADC1(Us) ===
    UPDATE_IF_CHANGED(adc1_val, last_disp_adc1, CHANGE_THRESHOLD, {
        LCD_Fill(260, 35, 315, 55, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 35, adc1_val, 4, 16);
    });

    // === ADC2(Ui) ===
    UPDATE_IF_CHANGED(adc2_val, last_disp_adc2, CHANGE_THRESHOLD, {
        LCD_Fill(260, 58, 315, 78, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 58, adc2_val, 4, 16);
    });

    // === ADC3(load) ===
    UPDATE_IF_CHANGED(adc3_val1, last_disp_adc3_1, CHANGE_THRESHOLD, {
        LCD_Fill(260, 81, 315, 101, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 81, adc3_val1, 4, 16);
    });

    // === ADC3(open) ===
    UPDATE_IF_CHANGED(adc3_val2, last_disp_adc3_2, CHANGE_THRESHOLD, {
        LCD_Fill(260, 104, 315, 124, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 104, adc3_val2, 4, 16);
    });

    // === ADC3(DC) ===
    UPDATE_IF_CHANGED(adc3_valDC, last_disp_adc3_dc, CHANGE_THRESHOLD, {
        LCD_Fill(260, 127, 315, 147, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 127, adc3_valDC, 4, 16);
    });

    // === Rin, Rout, Av（计算结果，变化>20%才刷新） ===
    static float last_disp_rin = -9999.0f;
    static float last_disp_rout = -9999.0f;
    static float last_disp_av = -9999.0f;

    // Rin
    {
        float diff = input_resistance - last_disp_rin;
        if(diff < 0) diff = -diff;
        if(last_disp_rin < -1000 || diff > 20.0f)
        {
            POINT_COLOR = WHITE;
            LCD_ShowString(55, 35, 72, 22, 16, (uint8_t *)"       ");
            POINT_COLOR = BLUE;
            if(input_resistance >= 1000.0f)
                LCD_ShowNum(55, 35, (uint32_t)(input_resistance), 4, 16);
            else
            {
                uint32_t ri_int = (uint32_t)(input_resistance);
                uint32_t ri_dec = (uint32_t)(input_resistance * 10) % 10;
                LCD_ShowNum(55, 35, ri_int, 3, 16);
                LCD_ShowString(78, 35, 10, 22, 16, (uint8_t *)".");
                LCD_ShowNum(84, 35, ri_dec, 1, 16);
            }
            last_disp_rin = input_resistance;
        }
    }

    // Rout
    {
        float diff = output_resistance - last_disp_rout;
        if(diff < 0) diff = -diff;
        if(last_disp_rout < -1000 || diff > 20.0f)
        {
            POINT_COLOR = WHITE;
            LCD_ShowString(55, 58, 72, 22, 16, (uint8_t *)"       ");
            POINT_COLOR = BLUE;
            if(output_resistance >= 1000.0f)
                LCD_ShowNum(55, 58, (uint32_t)(output_resistance), 4, 16);
            else
            {
                uint32_t ro_int = (uint32_t)(output_resistance);
                uint32_t ro_dec = (uint32_t)(output_resistance * 10) % 10;
                LCD_ShowNum(55, 58, ro_int, 3, 16);
                LCD_ShowString(78, 58, 10, 22, 16, (uint8_t *)".");
                LCD_ShowNum(84, 58, ro_dec, 1, 16);
            }
            last_disp_rout = output_resistance;
        }
    }

    // Av
    {
        float diff = amplification - last_disp_av;
        if(diff < 0) diff = -diff;
        if(last_disp_av < -1000 || diff > 0.5f)
        {
            POINT_COLOR = WHITE;
            LCD_ShowString(55, 81, 72, 22, 16, (uint8_t *)"       ");
            POINT_COLOR = BLUE;
            uint32_t av_int = (uint32_t)(amplification);
            uint32_t av_dec = (uint32_t)(amplification * 10) % 10;
            LCD_ShowNum(55, 81, av_int, 3, 16);
            LCD_ShowString(78, 81, 10, 22, 16, (uint8_t *)".");
            LCD_ShowNum(84, 81, av_dec, 1, 16);
            last_disp_av = amplification;
        }
    }

    #undef CHANGE_THRESHOLD
}

// ============================================================
//  界面0：开始/停止
// ============================================================
void Page0_StartMeasure(void)
{
    measure_running = !measure_running;
    if(measure_running)
    {
        POINT_COLOR = GREEN;
        LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"RUN ");
    }
    else
    {
        POINT_COLOR = BLUE;
        LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");
    }
}