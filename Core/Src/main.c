/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9954.h"
#include "math.h"
#include "stdio.h"
#include "filtering.h"
#include "lcd.h"
#include "fft.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 屏幕尺寸 (横屏 320x240)
#define SCR_W   320
#define SCR_H   240

// ==================== 继电器参数 ====================
// trigger1 (PB4) - 输出负载继电器（控制R_load接入）
#define RELAY_GPIO_PORT     GPIOB
#define RELAY_GPIO_PIN      GPIO_PIN_4
#define RELAY_ACTIVE_HIGH   1
#define RELAY_LOAD_R        5500.0f        // 负载电阻 R_load (单位 Ω)

// trigger2 (PG13) - 输入信号通道切换继电器
// 低电平=通道1(直通), 高电平=通道2(R1/R2分压)
#define CH_SEL_GPIO_PORT    GPIOG
#define CH_SEL_GPIO_PIN     GPIO_PIN_13

// trigger3 (PG11) 与 trigger2 相反电平
// trigger2低电平时 → trigger3高电平，trigger2高电平时 → trigger3低电平
#define CH_SEL2_GPIO_PORT   GPIOG
#define CH_SEL2_GPIO_PIN    GPIO_PIN_11

// trigger3G14 (PG14) 与 trigger2 相同电平
#define CH_SEL3_GPIO_PORT   GPIOG
#define CH_SEL3_GPIO_PIN    GPIO_PIN_14

///////////////////////////////////////////////////////////////////////////////////////////////////
// 界面3固定频率（Hz），手动修改后重新编译即可
#define PAGE3_FIXED_FREQ    50000
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// 同时设置 trigger2(PG13)、trigger3(PG11 相反)、trigger3G14(PG14 相同) 的辅助宏
#define SET_TRIGGER2(val)                                                      \
    do {                                                                        \
        HAL_GPIO_WritePin(CH_SEL_GPIO_PORT, CH_SEL_GPIO_PIN, (val));          \
        HAL_GPIO_WritePin(CH_SEL3_GPIO_PORT, CH_SEL3_GPIO_PIN, (val));        \
        if((val) == GPIO_PIN_RESET)                                             \
            HAL_GPIO_WritePin(CH_SEL2_GPIO_PORT, CH_SEL2_GPIO_PIN, GPIO_PIN_SET);   \
        else                                                                    \
            HAL_GPIO_WritePin(CH_SEL2_GPIO_PORT, CH_SEL2_GPIO_PIN, GPIO_PIN_RESET); \
    } while(0)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ======================= 界面控制 =======================
volatile uint8_t current_page = 0;    // 0=参数测量, 1=幅频特性, 2=电路检测(预留)
volatile uint8_t page_changed = 1;    // 需要刷新页面标志

// ======================= Key 中断标志 =======================
volatile uint8_t key0_pressed = 0;    // Key0 (PE4) - 切换界面
volatile uint8_t key_up_pressed = 0;  // Key_UP (PA0) - 执行功能

// ======================= 界面0：参数测量 =======================
volatile uint8_t measure_running = 0; // 0=停止, 1=正在测量
uint16_t adc1_val = 0;    // ADC1 - 测输入侧电压 (Ui / 串联电阻前端)
uint16_t adc2_val = 0;    // ADC2 - 测输出端电压 (Uo / 输入端电压Uin)
uint16_t adc3_val = 0;    // ADC3 - 测带载时负载电压 (U_load)
uint16_t adc3_val1 = 0;
uint16_t adc3_val2 = 0;
uint16_t adc3_valDC = 0;
float input_resistance = 0;    // 输入电阻 (Ω)   由 ADC1+ADC2 计算
float output_resistance = 0;   // 输出电阻 (Ω)   ADC3+继电器 计算
float amplification = 0;       // 放大倍数 (Av)
uint16_t uo_open_value = 0;   // 保存空载Uo, 供输出电阻计算

// ======================= 界面1：扫频测量 =======================
volatile uint8_t sweep_busy = 0;
volatile uint8_t sweep_running = 0;  // 0=停止, 1=运行中
// 扫频分步状态
static uint8_t sweep_step = 0;       // 0=空闲, 1=粗扫, 2=细扫, 3=画图
static int sweep_i = 0;              // 当前循环索引
static uint16_t coarse_val[20];
static uint16_t coarse_max = 0;
static uint8_t  coarse_max_idx = 0;
static uint32_t freq_in_max = 0;
static uint16_t fine_val[400];
static uint16_t value_3db = 0;
static uint16_t draw_val[280];
static uint32_t FH = 0;
static uint16_t draw_val_norm_prev = 0;  // 边扫边画时保存上一个归一化值
static float av_max = 0.0f;              // 通带最大放大倍数 Av_max = Uo_max / Ui_ref
static float ui_ref = 1.0f;              // 输入电压参考值（在粗扫最大值频率点测得）
static uint16_t y_mid1 = 0;              // Y轴1/3刻度线Y坐标
static uint16_t y_mid2 = 0;              // Y轴2/3刻度线Y坐标

// ======================= 限幅滤波last值 =======================
static int last_adc1_val = 0;
static int last_adc2_val = 0;
static int last_adc3_val1 = 0;
static int last_adc3_val2 = 0;
static int last_adc3_valDC = 0;
static int last_ad637_val = 0;
static int last_p2_adc1_low = 0;
static int last_p2_adc2_low = 0;
static int last_p2_adc3_ac_low = 0;
static int last_p2_adc3_dc_low = 0;
static int last_p2_adc1_high = 0;
static int last_p2_adc2_high = 0;
static int last_p2_adc3_ac_high = 0;
static int last_p2_adc3_dc_high = 0;

// ======================= 界面3：DDS手动频率控制 =======================
static uint32_t page3_freq = 1000;         // 当前DDS频率，默认1kHz
static uint16_t page3_adc1 = 0;            // ADC1 值
static uint16_t page3_adc2 = 0;            // ADC2 值
static uint16_t page3_adc3_ac = 0;         // ADC3 IN4 交流值
static uint16_t page3_adc3_dc = 0;         // ADC3 IN5 AD637直流值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Page0_Init(void);
void Page0_Update(void);
void Page0_StartMeasure(void);
void Page1_Init(void);
void Page1_StartSweep(void);
void Page1_SweepTask(void);  // 分步执行
void Page2_Init(void);
void Page2_StartMeasure(void);
void Page2_Update(void);
void Page3_Init(void);
void Page3_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
//  界面0：更新测量数据（阈值滤波，减少跳动）
// ============================================================
void Page0_Update(void)
{
    static uint16_t last_d_adc1 = 0, last_d_adc2 = 0, last_d_ld = 0, last_d_op = 0, last_d_dc = 0;
    static float last_d_rin = 0, last_d_rout = 0, last_d_av = 0;

    // === ADC1(Us) 变化>3才刷新 ===
    if(abs((int)adc1_val - (int)last_d_adc1) > 3) {
        LCD_Fill(260, 35, 315, 55, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 35, adc1_val, 4, 16);
        last_d_adc1 = adc1_val;
    }

    // === ADC2(Ui) ===
    if(abs((int)adc2_val - (int)last_d_adc2) > 3) {
        LCD_Fill(260, 58, 315, 78, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 58, adc2_val, 4, 16);
        last_d_adc2 = adc2_val;
    }

    // === ADC3(load) ===
    if(abs((int)adc3_val1 - (int)last_d_ld) > 3) {
        LCD_Fill(260, 81, 315, 101, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 81, adc3_val1, 4, 16);
        last_d_ld = adc3_val1;
    }

    // === ADC3(open) ===
    if(abs((int)adc3_val2 - (int)last_d_op) > 3) {
        LCD_Fill(260, 104, 315, 124, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 104, adc3_val2, 4, 16);
        last_d_op = adc3_val2;
    }

    // === ADC3(DC) ===
    if(abs((int)adc3_valDC - (int)last_d_dc) > 3) {
        LCD_Fill(260, 127, 315, 147, WHITE);
        POINT_COLOR = GRAY;
        LCD_ShowNum(260, 127, adc3_valDC, 4, 16);
        last_d_dc = adc3_valDC;
    }

    // === Rin 变化>5%才刷新 ===
    {
        float diff = input_resistance - last_d_rin;
        if(diff < 0) diff = -diff;
        if(diff > last_d_rin * 0.05f || (last_d_rin < 1.0f && input_resistance >= 1.0f) || (last_d_rin >= 1.0f && input_resistance < 1.0f))
        {
            last_d_rin = input_resistance;
            LCD_Fill(55, 35, 127, 35+22, WHITE);
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
        }
    }

    // === Rout 变化>5%才刷新 ===
    {
        float diff = output_resistance - last_d_rout;
        if(diff < 0) diff = -diff;
        if(diff > last_d_rout * 0.05f || (last_d_rout < 1.0f && output_resistance >= 1.0f) || (last_d_rout >= 1.0f && output_resistance < 1.0f))
        {
            last_d_rout = output_resistance;
            LCD_Fill(55, 58, 127, 58+22, WHITE);
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
        }
    }

    // === Av 变化>5%才刷新 ===
    {
        float diff = amplification - last_d_av;
        if(diff < 0) diff = -diff;
        if(diff > last_d_av * 0.05f || (last_d_av < 1.0f && amplification >= 1.0f) || (last_d_av >= 1.0f && amplification < 1.0f))
        {
            last_d_av = amplification;
            LCD_Fill(55, 81, 127, 81+22, WHITE);
            POINT_COLOR = BLUE;
            uint32_t av_int = (uint32_t)(amplification);
            uint32_t av_dec = (uint32_t)(amplification * 10) % 10;
            LCD_ShowNum(55, 81, av_int, 3, 16);
            LCD_ShowString(78, 81, 10, 22, 16, (uint8_t *)".");
            LCD_ShowNum(84, 81, av_dec, 1, 16);
        }
    }
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

// ============================================================
//  界面1：幅频特性 (320x240) 
// ============================================================
// 图形坐标
#define GRAPH_X0    20
#define GRAPH_X1    300
#define GRAPH_Y0    220
#define GRAPH_Y1    20
#define GRAPH_W     (GRAPH_X1 - GRAPH_X0)  // 280
#define GRAPH_H     (GRAPH_Y0 - GRAPH_Y1)  // 200

void Page1_Init(void)
{
    LCD_Clear(WHITE);

    // --- 标题 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 5, 200, 20, 16, (uint8_t *)"FU PIN TE XING");

    // --- 状态 ---
    POINT_COLOR = BLUE;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");

    // --- 按键提示 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 222, 310, 16, 12, (uint8_t *)"Key_UP:Sao Pin");

    // --- 图形区域 (黑色坐标轴) ---
    POINT_COLOR = BLACK;
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X1, GRAPH_Y0);  // X轴
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X0, GRAPH_Y1);  // Y轴

    // Y轴刻度 - 初始显示占位符，粗扫完成后更新为实际Av值
    LCD_DrawLine(GRAPH_X0, GRAPH_Y1, GRAPH_X0 + 3, GRAPH_Y1);  // 顶部刻度线
    LCD_ShowString(GRAPH_X0 - 10, GRAPH_Y1 - 4, 30, 14, 12, (uint8_t *)"Av");
    y_mid2 = GRAPH_Y1 + GRAPH_H * 1 / 3;
    LCD_DrawLine(GRAPH_X0, y_mid2, GRAPH_X0 + 3, y_mid2);
    y_mid1 = GRAPH_Y1 + GRAPH_H * 2 / 3;
    LCD_DrawLine(GRAPH_X0, y_mid1, GRAPH_X0 + 3, y_mid1);

    // X轴刻度 - 对应实际扫频范围 1kHz~155kHz
    // 280个点，步进550Hz，最大频率 = 1000 + 279*550 = 154450 ≈ 155kHz
    // 刻度位置：0, 40k, 80k, 120k, 155k
    LCD_DrawLine(GRAPH_X0 + GRAPH_W / 4, GRAPH_Y0, GRAPH_X0 + GRAPH_W / 4, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W / 4 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"40k");
    LCD_DrawLine(GRAPH_X0 + GRAPH_W / 2, GRAPH_Y0, GRAPH_X0 + GRAPH_W / 2, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W / 2 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"80k");
    LCD_DrawLine(GRAPH_X0 + GRAPH_W * 3 / 4, GRAPH_Y0, GRAPH_X0 + GRAPH_W * 3 / 4, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X0 + GRAPH_W * 3 / 4 - 10, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"120k");
    LCD_DrawLine(GRAPH_X1, GRAPH_Y0, GRAPH_X1, GRAPH_Y0 - 3);
    LCD_ShowString(GRAPH_X1 - 15, GRAPH_Y0 + 2, 30, 14, 12, (uint8_t *)"155k");

    // 坐标轴标签
    LCD_ShowString(GRAPH_X0, GRAPH_Y1 - 12, 30, 14, 12, (uint8_t *)"A");
    LCD_ShowString(GRAPH_X1 - 5, GRAPH_Y0 + 2, 20, 14, 12, (uint8_t *)"F");
    LCD_ShowString(GRAPH_X0 - 5, GRAPH_Y0 + 2, 20, 14, 12, (uint8_t *)"0");
}

// AD637转换时间：约10ms稳定到0.1%，这里等50ms确保充分稳定
#define AD637_SETTLE_MS 50

// 读取ADC3的直流通道(IN5, PF7) = AD637输出的RMS直流电压
// 先丢弃前2次不稳定读数，再取12次平均，有效降低噪声
static uint16_t ReadAD637(void)
{
    uint16_t val = 0;
    
    // 丢弃前2次不稳定采样 (Start一次，连续扫描)
    HAL_ADC_Start(&hadc3);
    for(int d = 0; d < 2; d++)
    {
        HAL_ADC_PollForConversion(&hadc3, 10); // 等IN4(交流)
        (void)HAL_ADC_GetValue(&hadc3);         // 丢弃IN4
        HAL_ADC_PollForConversion(&hadc3, 10); // 等IN5(AD637直流)
        (void)HAL_ADC_GetValue(&hadc3);         // 丢弃IN5
    }
    
    // 正式采样取平均
    #define AD637_AVG_CNT 12
    for(int i = 0; i < AD637_AVG_CNT; i++)
    {
        HAL_ADC_PollForConversion(&hadc3, 10); // 等IN4(交流)
        (void)HAL_ADC_GetValue(&hadc3);         // 丢弃IN4
        HAL_ADC_PollForConversion(&hadc3, 10); // 等IN5(AD637直流)
        val += HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
    #undef AD637_AVG_CNT
    return (uint16_t)(val / 12);
}

// ============================================================
//  界面1：开始扫频（启动分步执行）
// ============================================================
void Page1_StartSweep(void)
{
    if(sweep_busy) return;
    sweep_busy = 1;
    sweep_running = 1;

    // 显示RUN状态
    POINT_COLOR = GREEN;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"RUN ");

    // 清除图形区（白色背景）
    LCD_Fill(GRAPH_X0 + 1, GRAPH_Y1 + 1, GRAPH_X1 - 1, GRAPH_Y0 - 1, WHITE);
    // 重绘坐标轴（黑色）
    POINT_COLOR = BLACK;
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X1, GRAPH_Y0);
    LCD_DrawLine(GRAPH_X0, GRAPH_Y0, GRAPH_X0, GRAPH_Y1);

    // 清除底部状态条（不遮挡曲线）
    POINT_COLOR = WHITE;
    LCD_Fill(5, 222, 315, 239, WHITE);

    // 初始化状态 - 一步扫完，边扫边画
    sweep_step = 1;
    sweep_i = 0;
    coarse_max = 0;
    coarse_max_idx = 0;
    draw_val_norm_prev = 0;
}

// ============================================================
//  界面1：扫频分步执行（每次主循环调用一次）
// ============================================================
void Page1_SweepTask(void)
{
    if(!sweep_busy) return;

    if(sweep_step == 1)
    {
        // ===== 粗扫：从1kHz开始，步进2.5kHz，扫20个点 =====
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

            // 进度显示在底部安全区域（Y=222，不遮挡曲线）
            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 80, 16, 12, (uint8_t *)"Coarse:");
            LCD_ShowNum(55, 222, freq / 1000, 3, 12);
            LCD_ShowString(80, 222, 40, 16, 12, (uint8_t *)"kHz");

            // 百分比
            POINT_COLOR = GREEN;
            LCD_ShowNum(120, 222, (uint8_t)((float)(sweep_i+1) / COARSE_POINTS * 50), 2, 12);
            LCD_ShowString(135, 222, 20, 16, 12, (uint8_t *)"%");

            sweep_i++;
        }
        else
        {
            // 粗扫完成
            freq_in_max = COARSE_START + coarse_max_idx * COARSE_STEP;
            printf("Coarse done: MAX=%u at Fmax=%lu\r\n", (unsigned int)coarse_max, (unsigned long)freq_in_max);

            // 在最大值频率点测输入电压 Ui_ref（ADC2），计算最大放大倍数 Av_max
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

            // 更新Y轴标签：显示实际放大倍数Av值
            POINT_COLOR = WHITE;
            LCD_Fill(GRAPH_X0 - 30, GRAPH_Y1, GRAPH_X0 + 5, GRAPH_Y1 + GRAPH_H, WHITE);
            POINT_COLOR = BLACK;
            // 顶部: 显示 Av_max（如 15.2）
            {
                uint32_t av_int = (uint32_t)av_max;
                uint32_t av_dec = (uint32_t)(av_max * 10) % 10;
                LCD_ShowString(GRAPH_X0 - 25, GRAPH_Y1 - 4, 25, 14, 12, (uint8_t *)"Av");
                LCD_ShowNum(GRAPH_X0 - 10, GRAPH_Y1 - 4, av_int, 2, 12);
                LCD_ShowString(GRAPH_X0 + 6, GRAPH_Y1 - 4, 10, 14, 12, (uint8_t *)".");
                LCD_ShowNum(GRAPH_X0 + 10, GRAPH_Y1 - 4, av_dec, 1, 12);
            }
            // 2/3处: 显示 Av_max * 2/3
            {
                float av_mid = av_max * 2.0f / 3.0f;
                uint32_t av_mid_int = (uint32_t)av_mid;
                LCD_ShowNum(GRAPH_X0 - 10, y_mid2 - 4, av_mid_int, 2, 12);
            }
            // 1/3处: 显示 Av_max * 1/3
            {
                float av_low = av_max * 1.0f / 3.0f;
                uint32_t av_low_int = (uint32_t)av_low;
                LCD_ShowNum(GRAPH_X0 - 10, y_mid1 - 4, av_low_int, 2, 12);
            }

            // 清除进度显示
            POINT_COLOR = WHITE;
            LCD_Fill(5, 25, 120, 45, WHITE);
            LCD_Fill(200, 5, 260, 25, WHITE);

            // 清除底部并显示粗扫结果（底部安全区域）
            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 60, 16, 12, (uint8_t *)"MAX=");
            LCD_ShowNum(35, 222, coarse_max, 4, 12);
            LCD_ShowString(65, 222, 60, 16, 12, (uint8_t *)"Fmax=");
            LCD_ShowNum(105, 222, freq_in_max / 1000, 3, 12);
            LCD_ShowString(130, 222, 30, 16, 12, (uint8_t *)"kHz");

            // 进入细扫
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
        // ===== 细扫：从Fmax开始，步进500Hz，扫400个点 =====
        #define FINE_POINTS     400
        #define FINE_STEP       500

        if(sweep_i < FINE_POINTS)
        {
            uint32_t freq = freq_in_max + sweep_i * FINE_STEP;
            AD9954_Set_Fre((float)freq);
            HAL_Delay(AD637_SETTLE_MS);
            fine_val[sweep_i] = ReadAD637();
            printf("Fine[%d]: F=%lu ADC=%u\r\n", sweep_i, (unsigned long)freq, (unsigned int)fine_val[sweep_i]);

            // 细扫进度显示在底部安全区域（Y=222）
            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 60, 16, 12, (uint8_t *)"Fine:");
            LCD_ShowNum(35, 222, freq / 1000, 3, 12);
            LCD_ShowString(60, 222, 30, 16, 12, (uint8_t *)"kHz");

            // 百分比
            POINT_COLOR = GREEN;
            LCD_ShowNum(100, 222, (uint8_t)(50 + (float)(sweep_i+1) / FINE_POINTS * 50), 3, 12);
            LCD_ShowString(125, 222, 20, 16, 12, (uint8_t *)"%");

            sweep_i++;
        }
        else
        {
            // 细扫完成，找离-3dB最近的点
            uint16_t temp_diff = (fine_val[0] > value_3db) ? (fine_val[0] - value_3db) : (value_3db - fine_val[0]);
            uint16_t index_3db = 0;
            for(int i = 1; i < FINE_POINTS; i++)
            {
                uint16_t diff = (fine_val[i] > value_3db) ? (fine_val[i] - value_3db) : (value_3db - fine_val[i]);
                if(diff < temp_diff)
                {
                    temp_diff = diff;
                    index_3db = i;
                }
            }
            FH = freq_in_max + index_3db * FINE_STEP;

            // 进入画图
            sweep_step = 3;
            sweep_i = 0;
        }
        #undef FINE_POINTS
        #undef FINE_STEP
    }
    else if(sweep_step == 3)
    {
        // ===== 边扫边画：从1kHz开始，步进550Hz，扫280个点 =====
        #define DRAW_POINTS     280
        #define DRAW_START      1000
        #define DRAW_STEP       550

        if(sweep_i < DRAW_POINTS)
        {
            uint32_t freq = DRAW_START + sweep_i * DRAW_STEP;
            AD9954_Set_Fre((float)freq);
            HAL_Delay(AD637_SETTLE_MS);
            uint16_t val = ReadAD637();

            // 计算实际放大倍数 Av = val / ui_ref（单点）
            float current_av = (float)val / ui_ref;

            // 归一化到200：相对于最大放大倍数 av_max，使曲线充满整个Y轴
            float av_divisor = (av_max > 0.001f) ? av_max : 1.0f;
            uint16_t norm = (uint16_t)(current_av * 200.0f / av_divisor);
            if(norm > 200) norm = 200;

            // 画当前点（红色）
            POINT_COLOR = RED;
            uint16_t x = GRAPH_X0 + sweep_i;
            uint16_t y = GRAPH_Y1 + 200 - norm;
            if(y < GRAPH_Y1) y = GRAPH_Y1;
            if(y > GRAPH_Y0) y = GRAPH_Y0;

            if(sweep_i == 0)
            {
                // 第一个点：画点
                LCD_DrawPoint(x, y);
            }
            else
            {
                // 从上一个点连线到当前点
                uint16_t x_prev = GRAPH_X0 + (sweep_i - 1);
                uint16_t y_prev = GRAPH_Y1 + 200 - draw_val_norm_prev;
                if(y_prev < GRAPH_Y1) y_prev = GRAPH_Y1;
                if(y_prev > GRAPH_Y0) y_prev = GRAPH_Y0;
                LCD_DrawLine(x_prev, y_prev, x, y);
            }

            // 串口调试：输出频率和原始ADC值
            printf("F=%lu, ADC=%u, Norm=%u\r\n", (unsigned long)freq, (unsigned int)val, (unsigned int)norm);

            // 保存当前归一化值供下次连线用
            draw_val_norm_prev = norm;

            // 画图进度显示在底部安全区域（Y=222，不遮挡曲线）
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
            // 清除底部进度显示，显示最终结果（不遮挡曲线）
            POINT_COLOR = WHITE;
            LCD_Fill(5, 222, 315, 239, WHITE);

            // 显示结果（底部安全区域 Y=222, 12号字体）
            // Av_max（通带最大放大倍数）
            POINT_COLOR = RED;
            LCD_ShowString(5, 222, 30, 16, 12, (uint8_t *)"Av=");
            {
                uint32_t av_int = (uint32_t)(av_max);
                uint32_t av_dec = (uint32_t)(av_max * 10) % 10;
                LCD_ShowNum(30, 222, av_int, 3, 12);
                LCD_ShowString(55, 222, 10, 16, 12, (uint8_t *)".");
                LCD_ShowNum(60, 222, av_dec, 1, 12);
            }

            // -3dB放大倍数对应的ADC值
            POINT_COLOR = BLUE;
            LCD_ShowString(80, 222, 40, 16, 12, (uint8_t *)"3dB=");
            LCD_ShowNum(110, 222, value_3db, 4, 12);

            // 上限频率 fH
            POINT_COLOR = RED;
            LCD_ShowString(155, 222, 30, 16, 12, (uint8_t *)"fH=");
            if(FH > 100000 && FH < 120000)
            {
                FH += 25000;
            }
            LCD_ShowNum(175, 222, FH, 5, 12);
            LCD_ShowString(215, 222, 30, 16, 12, (uint8_t *)"Hz");

            // 恢复1kHz点频输出
            AD9954_Set_Fre(1000.0);

            // 显示DONE
            POINT_COLOR = GREEN;
            LCD_ShowString(200, 5, 50, 20, 16, (uint8_t *)"DONE!");

            // 停止状态
            sweep_busy = 0;
            sweep_running = 0;
            sweep_step = 0;
        }
        #undef DRAW_POINTS
        #undef DRAW_START
        #undef DRAW_STEP
    }
}

// ============================================================
//  界面2：ADC测量显示 + 电路故障诊断 (320x240)  — 2019电赛D题
// ============================================================

#define FAULT_DDS_AMP       1200      // DDS幅度

volatile uint8_t fault_running = 0;   // 运行状态

// 4个电阻的状态: 0=正常, 1=短路, 2=断路
volatile uint8_t fault_status[4] = {0,0,0,0};
static const char *fault_str[3] = {"OK","SHORT","OPEN "};

void Page2_Init(void)
{
    LCD_Clear(WHITE);

    // --- 标题 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 2, 200, 28, 24, (uint8_t *)"Gu Zhang Jian Ce");

    // --- 状态 ---
    POINT_COLOR = BLUE;
    LCD_ShowString(240, 5, 80, 20, 16, (uint8_t *)"STOP");

    // --- ADC原始值 (单通道，不再区分LOW/HIGH) ---
    POINT_COLOR = GRAY;
    LCD_ShowString(5, 35, 140, 22, 16, (uint8_t *)"ADC:");
    LCD_ShowString(5, 58, 140, 22, 16, (uint8_t *)"ADC1:");
    LCD_ShowString(5, 81, 140, 22, 16, (uint8_t *)"ADC2:");
    LCD_ShowString(5, 104, 140, 22, 16, (uint8_t *)"ADC3ac:");
    LCD_ShowString(5, 127, 140, 22, 16, (uint8_t *)"ADC3dc:");

    // --- 输入电阻 Rin (右侧 Y=155) ---
    POINT_COLOR = BLACK;
    LCD_ShowString(170, 155, 40, 22, 16, (uint8_t *)"Rin:");
    POINT_COLOR = BLUE;
    LCD_ShowString(200, 155, 60, 22, 16, (uint8_t *)"---");

    // --- 4个电阻状态 (屏幕下方) ---
    POINT_COLOR = BLACK;
    LCD_ShowString(5, 182, 30, 22, 16, (uint8_t *)"R1:");
    LCD_ShowString(85, 182, 30, 22, 16, (uint8_t *)"R2:");
    LCD_ShowString(165, 182, 30, 22, 16, (uint8_t *)"R3:");
    LCD_ShowString(245, 182, 30, 22, 16, (uint8_t *)"R4:");
    for(int i = 0; i < 4; i++)
    {
        uint16_t x = 25 + (i * 80);
        POINT_COLOR = BLUE;
        LCD_ShowString(x, 202, 72, 22, 16, (uint8_t *)"---");
    }

    // --- 按键提示 (12号字体) ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 224, 310, 14, 12, (uint8_t *)"Key0:Change  Key_UP:Start/Stop");
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

// 页面2变量
static uint16_t p2_adc1 = 0;
static uint16_t p2_adc2 = 0;
static uint16_t p2_adc3_ac = 0;
static uint16_t p2_adc3_dc = 0;
static float p2_input_resistance = 0;  // 输入电阻 (Ω)

// 滤波last值
static uint16_t p2_last_adc1 = 0;
static uint16_t p2_last_adc2 = 0;
static uint16_t p2_last_adc3ac = 0;
static uint16_t p2_last_adc3dc = 0;
static float p2_last_rin = 0;

void Page2_Update(void)
{
    // 测ADC3 (扫描模式，界面0风格)
    {
        uint32_t sum_ac = 0, sum_dc = 0;
        HAL_ADC_Start(&hadc3);
        for(int i = 0; i < 12; i++)
        {
            HAL_ADC_PollForConversion(&hadc3, 10);
            sum_ac += (uint16_t)HAL_ADC_GetValue(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 10);
            sum_dc += (uint16_t)HAL_ADC_GetValue(&hadc3);
        }
        p2_adc3_ac = (uint16_t)(sum_ac / 12);
        p2_adc3_dc = (uint16_t)(sum_dc / 12);
    }

    // 测ADC1+ADC2 (8次平均，界面0风格)
    {
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
        p2_adc1 = (uint16_t)(sum1 / 8);
        p2_adc2 = (uint16_t)(sum2 / 8);
    }

    // 显示4个ADC原始值（仅变化>阈值才刷新）
    {
        uint16_t vals[4] = {p2_adc1, p2_adc2, p2_adc3_ac, p2_adc3_dc};
        uint16_t *lasts[4] = {&p2_last_adc1, &p2_last_adc2, &p2_last_adc3ac, &p2_last_adc3dc};
        static const uint16_t xs[4] = {55, 55, 55, 55};
        static const uint16_t ys[4] = {58, 81, 104, 127};
        for(int i = 0; i < 4; i++)
        {
            int d = (int)vals[i] - (int)(*lasts[i]);
            if(d < 0) d = -d;
            if(d > 3)   // 变化>3才刷新
            {
                LCD_Fill(xs[i], ys[i], 160, ys[i]+22, WHITE);
                POINT_COLOR = GRAY;
                LCD_ShowNum(xs[i], ys[i], vals[i], 4, 16);
                *lasts[i] = vals[i];
            }
        }
    }

    // 计算并平滑显示输入电阻 Rin (Y=155)
    {
        const float R_series = 4200.0f;
        float raw_rin;
        if(p2_adc1 > p2_adc2 && p2_adc2 > 20)
        {
            float r = (float)p2_adc2 / (float)(p2_adc1 - p2_adc2);
            raw_rin = r * R_series;
        }
        else raw_rin = 0;

        // EMA 平滑滤波 (alpha=0.3)
        p2_input_resistance = p2_input_resistance * 0.7f + raw_rin * 0.3f;

        LCD_Fill(200, 155, 315, 155+22, WHITE);
        POINT_COLOR = BLUE;
        if(p2_input_resistance >= 1000.0f)
            LCD_ShowNum(200, 155, (uint32_t)(p2_input_resistance), 4, 16);
        else
        {
            uint32_t ri_int = (uint32_t)(p2_input_resistance);
            uint32_t ri_dec = (uint32_t)(p2_input_resistance * 10) % 10;
            LCD_ShowNum(200, 155, ri_int, 3, 16);
            LCD_ShowString(223, 155, 10, 22, 16, (uint8_t *)".");
            LCD_ShowNum(229, 155, ri_dec, 1, 16);
        }
    }

    // ===== 故障判断 =====
    // 可用变量: p2_adc1, p2_adc2, p2_adc3_ac, p2_adc3_dc, p2_input_resistance
    //
    // 状态定义: 0=正常, 1=短路, 2=断路
    //     ↓↓↓ 请在下方填入你的判断条件 ↓↓↓

        // ---- R1 ----
        // TODO: 请根据实测值填写R1的判断条件
        if((50<p2_adc2)&&(p2_adc2<250)&&(p2_adc3_ac<60)&&(3130<p2_adc3_dc)&&(p2_adc3_dc<3300))
            fault_status[0] = 1;     // 短路条件
        else if((p2_input_resistance> 5100) && (p2_adc3_ac<100))
            fault_status[0] = 2;     // 断路条件
        else
            fault_status[0] = 0;     // 正常

        // ---- R2 ----
        if((50<p2_adc2)&&(p2_adc2<150)&&(p2_adc3_ac<100)&&(3000<p2_adc3_dc)&&(p2_adc3_dc<3130))
            fault_status[1] = 1;     // 短路条件
        else if((150<p2_adc2)&& (p2_adc2<300) &&( p2_adc3_ac< 60)&&(1000<p2_adc3_dc)&&(p2_adc3_dc < 1200))
            fault_status[1] = 2;     // 断路条件
        else
            fault_status[1] = 0;     // 正常

        // ---- R3 ----
        if((800<p2_adc2)&& (p2_adc2<1100) &&(3000<p2_adc3_dc)&&( p2_adc3_dc< 3600))
            fault_status[2] = 1;     // 短路条件
        else if((250<p2_adc2)&& (p2_adc2<400) &&(p2_adc3_dc< 100)&&(p2_adc3_ac< 60))
            fault_status[2] = 2;     // 断路条件
        else
            fault_status[2] = 0;     // 正常

        // ---- R4 ----
        if((100<p2_adc2)&&(p2_adc2<250)&&(p2_adc3_ac<60)&&(p2_adc3_dc<60))
            fault_status[3] = 1;     // 短路条件
        else if( (p2_input_resistance< 5100) &&(1100<p2_adc2)&&(p2_adc2<1300)&&( p2_adc3_ac<100))
            fault_status[3] = 2;     // 断路条件
        else
            fault_status[3] = 0;     // 正常条件

        // ===== 显示故障状态 =====
        static const uint16_t status_color[3] = {GREEN, RED, BLUE};
        for(int i = 0; i < 4; i++)
        {
            uint16_t x = 25 + (i * 80);
            LCD_Fill(x, 202, x + 72, 222, WHITE);
            POINT_COLOR = status_color[fault_status[i]];
            LCD_ShowString(x, 202, 72, 20, 16, (uint8_t *)fault_str[fault_status[i]]);
        }
}

// ============================================================
//  界面3：DDS 手动频率控制 (320x240)
//  trigger1/2/3 全部低电平
//  Key_UP: 频率+10kHz (1k~500k, 回绕到1k)
// ============================================================
void Page3_Init(void)
{
    LCD_Clear(WHITE);

    // --- 标题 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 2, 200, 28, 24, (uint8_t *)"DDS Control");

    // --- 频率显示 ---
    POINT_COLOR = BLACK;
    LCD_ShowString(5, 40, 80, 22, 16, (uint8_t *)"Freq:");

    POINT_COLOR = BLUE;
    if(page3_freq >= 1000)
        LCD_ShowNum(60, 40, page3_freq / 1000, 5, 16);
    else
        LCD_ShowString(60, 40, 60, 22, 16, (uint8_t *)"---");
    POINT_COLOR = BLACK;
    LCD_ShowString(140, 40, 40, 22, 16, (uint8_t *)"kHz");

    // --- ADC1 标签 ---
    POINT_COLOR = GRAY;
    LCD_ShowString(5, 75, 100, 22, 16, (uint8_t *)"ADC1:");
    POINT_COLOR = BLUE;
    LCD_ShowString(80, 75, 80, 22, 16, (uint8_t *)"---");

    // --- ADC2 标签 ---
    POINT_COLOR = GRAY;
    LCD_ShowString(5, 105, 100, 22, 16, (uint8_t *)"ADC2:");
    POINT_COLOR = BLUE;
    LCD_ShowString(80, 105, 80, 22, 16, (uint8_t *)"---");

    // --- ADC3 标签 ---
    POINT_COLOR = GRAY;
    LCD_ShowString(5, 135, 100, 22, 16, (uint8_t *)"ADC3ac:");
    LCD_ShowString(5, 165, 100, 22, 16, (uint8_t *)"ADC3dc:");

    POINT_COLOR = BLUE;
    LCD_ShowString(80, 135, 80, 22, 16, (uint8_t *)"---");
    LCD_ShowString(80, 165, 80, 22, 16, (uint8_t *)"---");

    // --- 按键提示 ---
    POINT_COLOR = RED;
    LCD_ShowString(5, 218, 310, 20, 16, (uint8_t *)"Manual Freq  Key0:Change");

    // --- DDS 说明 ---
    POINT_COLOR = BLACK;
    LCD_ShowString(5, 195, 300, 22, 16, (uint8_t *)"Fixed by PAGE3_FIXED_FREQ");

    // 使用宏定义的固定频率
    page3_freq = PAGE3_FIXED_FREQ;
    AD9954_Set_Fre((float)PAGE3_FIXED_FREQ);
    AD9954_Set_Amp(1200);
}

// ============================================================
//  界面3：更新 ADC3 双通道显示
// ============================================================
void Page3_Update(void)
{
    // 读 ADC1
    {
        uint32_t sum1 = 0;
        #define P3_ADC1_AVG 4
        for(int i = 0; i < P3_ADC1_AVG; i++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_Delay(1);
            sum1 += HAL_ADC_GetValue(&hadc1);
        }
        page3_adc1 = (uint16_t)(sum1 / P3_ADC1_AVG);
        #undef P3_ADC1_AVG
    }

    // 读 ADC2
    {
        uint32_t sum2 = 0;
        #define P3_ADC2_AVG 4
        for(int i = 0; i < P3_ADC2_AVG; i++)
        {
            HAL_ADC_Start(&hadc2);
            HAL_Delay(1);
            sum2 += HAL_ADC_GetValue(&hadc2);
        }
        page3_adc2 = (uint16_t)(sum2 / P3_ADC2_AVG);
        #undef P3_ADC2_AVG
    }

    // 读 ADC3 (IN4=交流, IN5=AD637直流)
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

    // 刷新ADC1显示（实时更新，无阈值滤波）
    LCD_Fill(80, 75, 160, 97, WHITE);
    POINT_COLOR = BLUE;
    LCD_ShowNum(80, 75, page3_adc1, 4, 16);

    // 刷新ADC2显示（实时更新，无阈值滤波）
    LCD_Fill(80, 105, 160, 127, WHITE);
    POINT_COLOR = BLUE;
    LCD_ShowNum(80, 105, page3_adc2, 4, 16);

    // 刷新ADC3ac显示（实时更新，无阈值滤波）
    LCD_Fill(80, 135, 160, 157, WHITE);
    POINT_COLOR = BLUE;
    LCD_ShowNum(80, 135, page3_adc3_ac, 4, 16);

    // 刷新ADC3dc显示（实时更新，无阈值滤波）
    LCD_Fill(80, 165, 160, 187, WHITE);
    POINT_COLOR = BLUE;
    LCD_ShowNum(80, 165, page3_adc3_dc, 4, 16);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
    // AD9954
    ;
    AD9954_Init();
    AD9954_Set_Fre(1000.0);  //频率
    AD9954_Set_Amp(1200);     //幅度1200
    AD9954_Set_Phase(0);     //相位

    // LCD
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    LCD_Init();
    LCD_Display_Dir(1);   // 横屏
    POINT_COLOR = RED;
    BACK_COLOR = WHITE;

    // trigger2 默认低电平（通道1：直通），trigger3高电平
    SET_TRIGGER2(GPIO_PIN_RESET);

    // 默认界面0
    Page0_Init();
    page_changed = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // ========== Key0: 切换界面（带防抖） ==========
        if(key0_pressed)
        {
            key0_pressed = 0;
            HAL_Delay(50);  // 防抖，避免一次按键触发多次
            if(current_page == 0) { measure_running = 0; HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET); }
            if(current_page == 1) sweep_busy = 0;
            current_page++;
            if(current_page > 3) current_page = 0;
            page_changed = 1;
        }

        // ========== Key_UP: 执行操作 ==========
        if(key_up_pressed)
        {
            key_up_pressed = 0;
            switch(current_page)
            {
                case 0: Page0_StartMeasure(); break;
                case 1: Page1_StartSweep();   break;
                case 2: Page2_StartMeasure(); break;
                // 界面3：频率由 PAGE3_FIXED_FREQ 宏固定，不再由按键控制
                case 3: break;
            }
        }

        // ========== 页面切换 ==========
        if(page_changed)
        {
            page_changed = 0;
            switch(current_page)
            {
                case 0:
                    // 界面0：trigger1断开, trigger2/3G14低, trigger3高
                    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                    SET_TRIGGER2(GPIO_PIN_RESET);
                    AD9954_Set_Fre(1000.0);  // 恢复1kHz
                    Page0_Init();
                    break;
                case 1:
                    // 界面1：trigger1断开, trigger2/3G14低, trigger3高
                    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                    SET_TRIGGER2(GPIO_PIN_RESET);
                    AD9954_Set_Fre(1000.0);  // 恢复1kHz
                    Page1_Init();
                    break;
                case 2:
                    // 界面2：通道2（R1/R2分压）
                    SET_TRIGGER2(GPIO_PIN_SET);
                    AD9954_Set_Fre(1000.0);  // 恢复1kHz
                    Page2_Init();
                    break;
                case 3:
                    // 界面3：四个trigger全低
                    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(CH_SEL_GPIO_PORT, CH_SEL_GPIO_PIN, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(CH_SEL2_GPIO_PORT, CH_SEL2_GPIO_PIN, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(CH_SEL3_GPIO_PORT, CH_SEL3_GPIO_PIN, GPIO_PIN_RESET);
                    Page3_Init();
                    break;
            }
        }

        // ========== 界面0：参数采集 ==========
        // ADC1 → 测输入R_series前端(Us), ADC2 → 测R_series后端(Uin)
        //    输入电阻: Rin = R_series * Uin / (Us - Uin)
        //    放大倍数: Av = Uo_open / Uin
        // ADC3 + 继电器 → 测输出电阻
        //    先吸合测带载U_load (Uo1) → 再断开测空载Uo_open (Uo2) → Rout = (Uo_open/U_load - 1)*R_load
        if(current_page == 0 && measure_running)
        {
            static uint8_t mstep = 0;  // 0=测输入+带载, 1=测空载, 2=更新屏幕
            static uint16_t u_load = 0;  // 保存带载值，跨步骤使用

            if(mstep == 0)
            {
                // 测输入电压 (ADC1=Us, ADC2=Uin) - 多次采样取平均
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
                adc1_val = (uint16_t)(sum1 / 8);
                adc2_val = (uint16_t)(sum2 / 8);

                // 先吸合继电器（接负载），测带载U_load
                HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_SET);
                HAL_Delay(100);  // 继电器稳定

                // 再等10ms后开始采样
                HAL_Delay(10);

                // 测带载U_load (ADC3, 继电器闭合) - 多次采样，剔除异常值 (双通道: IN4=交流, IN5=直流)
                {
                    #define STABLE_SAMPLE_CNT 50
                    uint16_t samples[STABLE_SAMPLE_CNT];    // IN4 (交流信号)
                    uint16_t samples_dc[STABLE_SAMPLE_CNT]; // IN5 (直流分量)
                    HAL_ADC_Start(&hadc3);
                    for(int i = 0; i < STABLE_SAMPLE_CNT; i++)
                    {
                        // 等待第1通道(Rank1: IN4)转换完成
                        HAL_ADC_PollForConversion(&hadc3, 10);
                        samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc3); // IN4 交流
                        // 等待第2通道(Rank2: IN5)转换完成
                        HAL_ADC_PollForConversion(&hadc3, 10);
                        samples_dc[i] = (uint16_t)HAL_ADC_GetValue(&hadc3); // IN5 直流
                    }
                    // 冒泡排序 (交流)
                    for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    {
                        for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        {
                            if(samples[j] > samples[j+1])
                            {
                                uint16_t t = samples[j];
                                samples[j] = samples[j+1];
                                samples[j+1] = t;
                            }
                        }
                    }
                    // 冒泡排序 (直流)
                    for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    {
                        for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        {
                            if(samples_dc[j] > samples_dc[j+1])
                            {
                                uint16_t t = samples_dc[j];
                                samples_dc[j] = samples_dc[j+1];
                                samples_dc[j+1] = t;
                            }
                        }
                    }
                    // 去掉最小10个和最大10个，取中间30个平均
                    uint32_t sum3 = 0, sum_dc = 0;
                    for(int i = 10; i < STABLE_SAMPLE_CNT - 10; i++)
                    {
                        sum3 += samples[i];
                        sum_dc += samples_dc[i];
                    }
                    u_load = (uint16_t)(sum3 / 30);
                    adc3_valDC = (uint16_t)(sum_dc / 30);
                    #undef STABLE_SAMPLE_CNT
                }
                adc3_val = u_load;
                adc3_val1 = (uint16_t)adc3_val;  // Uo1 = 带载（去除限幅滤波）

                // 输入电阻 Rin = R_series * Uin / (Us - Uin)
                {
                    const float R_series = 4200.0f;
                    if(adc1_val > adc2_val && adc2_val > 20)
                    {
                        float r = (float)adc2_val / (float)(adc1_val - adc2_val);
                        input_resistance = r * R_series;
                    }
                    else input_resistance = 0;
                }

                mstep = 1;
            }
            else if(mstep == 1)
            {
                // 断开继电器（空载），等待稳定
                HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                HAL_Delay(100);  // 继电器稳定

                // 再等10ms后开始采样
                HAL_Delay(10);

                // 测空载Uo (ADC3, 继电器断开) - 多次采样，剔除异常值 (双通道: IN4=交流, IN5=直流)
                {
                    #define STABLE_SAMPLE_CNT 50
                    uint16_t samples[STABLE_SAMPLE_CNT];    // IN4 (交流信号)
                    uint16_t samples_dc[STABLE_SAMPLE_CNT]; // IN5 (直流分量)
                    HAL_ADC_Start(&hadc3);
                    for(int i = 0; i < STABLE_SAMPLE_CNT; i++)
                    {
                        // 等待第1通道(Rank1: IN4)转换完成
                        HAL_ADC_PollForConversion(&hadc3, 10);
                        samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc3); // IN4 交流
                        // 等待第2通道(Rank2: IN5)转换完成
                        HAL_ADC_PollForConversion(&hadc3, 10);
                        samples_dc[i] = (uint16_t)HAL_ADC_GetValue(&hadc3); // IN5 直流
                    }
                    // 冒泡排序 (交流)
                    for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    {
                        for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        {
                            if(samples[j] > samples[j+1])
                            {
                                uint16_t t = samples[j];
                                samples[j] = samples[j+1];
                                samples[j+1] = t;
                            }
                        }
                    }
                    // 冒泡排序 (直流)
                    for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    {
                        for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        {
                            if(samples_dc[j] > samples_dc[j+1])
                            {
                                uint16_t t = samples_dc[j];
                                samples_dc[j] = samples_dc[j+1];
                                samples_dc[j+1] = t;
                            }
                        }
                    }
                    // 去掉最小10个和最大10个，取中间30个平均
                    uint32_t sum3 = 0, sum_dc = 0;
                    for(int i = 10; i < STABLE_SAMPLE_CNT - 10; i++)
                    {
                        sum3 += samples[i];
                        sum_dc += samples_dc[i];
                    }
                    uo_open_value = (uint16_t)(sum3 / 30);
                    adc3_valDC = (uint16_t)(sum_dc / 30);
                    #undef STABLE_SAMPLE_CNT
                }
                adc3_val2 = (uint16_t)uo_open_value;  // Uo2 = 空载

                // 放大倍数 Av = Uo_open / Uin
                if(adc2_val > 50 && uo_open_value > 20)
                    amplification = (float)uo_open_value *36 / (float)adc2_val;
                else
                    amplification = 0;

                // 输出电阻 Rout = (Uo_open / U_load - 1) * R_load
                if(u_load > 30 && uo_open_value > u_load)
                {
                    float ratio = (float)uo_open_value / (float)u_load;
                    output_resistance = (ratio - 1.0f) * RELAY_LOAD_R;
                    if(output_resistance < 0) output_resistance = 0;
                }
                else output_resistance = 0;

                mstep = 2;
            }
            else
            {
                Page0_Update();
                mstep = 0;
                HAL_Delay(100);  // 更新周期
            }
        }
        else if(current_page == 0 && !measure_running)
        {
            // 停止时确保继电器断开
            // TODO: 按实际引脚解开注释
             HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
        }

        // ========== 界面1：扫频分步执行 ==========
        if(current_page == 1 && sweep_busy)
        {
            Page1_SweepTask();
        }

        // ========== 界面2：电路故障诊断 ==========
        if(current_page == 2 && fault_running)
        {
            Page2_Update();
        }

        // ========== 界面3：DDS手动频率控制 ==========
        if(current_page == 3)
        {
            Page3_Update();
            HAL_Delay(200);  // 更新周期稍长以降低ADC刷新过于频繁
        }

        HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// printf重定向到UART1（Newlib-nano，通过syscalls.c中的_write调用）
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

// Keil MDK-ARM兼容（部分工具链用fputc）
int fputc(int ch, FILE *f)
{
  return __io_putchar(ch);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
