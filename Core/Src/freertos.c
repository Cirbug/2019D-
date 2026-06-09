/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "page0.h"
#include "page1.h"
#include "page2.h"
#include "page3.h"
#include "AD9954.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile uint8_t current_page;
extern volatile uint8_t page_changed;
extern volatile uint8_t key0_pressed;
extern volatile uint8_t key_up_pressed;
extern volatile uint8_t measure_running;
extern volatile uint8_t sweep_busy;
extern volatile uint8_t sweep_running;
extern volatile uint8_t fault_running;
extern volatile uint8_t fault_status[4];
extern uint16_t adc1_val;
extern uint16_t adc2_val;
extern uint16_t adc3_val;
extern uint16_t adc3_val1;
extern uint16_t adc3_val2;
extern uint16_t adc3_valDC;
extern float input_resistance;
extern float output_resistance;
extern float amplification;
extern uint16_t uo_open_value;
extern uint32_t page3_freq;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // ===== 初始化（原 main.c USER CODE BEGIN 2） =====
  AD9954_Init();
  AD9954_Set_Fre(1000.0);
  AD9954_Set_Amp(1200);
  AD9954_Set_Phase(0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  LCD_Init();
  LCD_Display_Dir(1);
  POINT_COLOR = RED;
  BACK_COLOR = WHITE;

  SET_TRIGGER2(GPIO_PIN_RESET);
  Page0_Init();
  page_changed = 0;

  /* Infinite loop */
  for(;;)
  {
    // ========== Key0: 切换界面（带防抖） ==========
    if(key0_pressed)
    {
        key0_pressed = 0;
        osDelay(50);
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
            case 3:
                page3_freq += 10000;
                if(page3_freq > 500000) page3_freq = 1000;
                AD9954_Set_Fre((float)page3_freq);
                {
                    LCD_Fill(60, 40, 135, 62, WHITE);
                    POINT_COLOR = BLUE;
                    LCD_ShowNum(60, 40, page3_freq / 1000, 5, 16);
                }
                break;
        }
    }

    // ========== 页面切换 ==========
    if(page_changed)
    {
        page_changed = 0;
        switch(current_page)
        {
            case 0:
                HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                SET_TRIGGER2(GPIO_PIN_RESET);
                AD9954_Set_Fre(1000.0);
                Page0_Init();
                break;
            case 1:
                HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                SET_TRIGGER2(GPIO_PIN_RESET);
                AD9954_Set_Fre(1000.0);
                Page1_Init();
                break;
            case 2:
                SET_TRIGGER2(GPIO_PIN_SET);
                AD9954_Set_Fre(1000.0);
                Page2_Init();
                break;
            case 3:
                HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CH_SEL_GPIO_PORT, CH_SEL_GPIO_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CH_SEL2_GPIO_PORT, CH_SEL2_GPIO_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CH_SEL3_GPIO_PORT, CH_SEL3_GPIO_PIN, GPIO_PIN_RESET);
                Page3_Init();
                break;
        }
    }

    // ========== 界面0：参数采集 ==========
    if(current_page == 0 && measure_running)
    {
        static uint8_t mstep = 0;
        static uint16_t u_load = 0;

        if(mstep == 0)
        {
            uint32_t sum1 = 0, sum2 = 0;
            for(int i = 0; i < 8; i++)
            {
                HAL_ADC_Start(&hadc1);
                osDelay(1);
                sum1 += HAL_ADC_GetValue(&hadc1);
                HAL_ADC_Start(&hadc2);
                osDelay(1);
                sum2 += HAL_ADC_GetValue(&hadc2);
            }
            adc1_val = (uint16_t)(sum1 / 8);
            adc2_val = (uint16_t)(sum2 / 8);

            HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_SET);
            osDelay(100);
            osDelay(10);

            {
                #define STABLE_SAMPLE_CNT 50
                uint16_t samples[STABLE_SAMPLE_CNT];
                uint16_t samples_dc[STABLE_SAMPLE_CNT];
                HAL_ADC_Start(&hadc3);
                for(int i = 0; i < STABLE_SAMPLE_CNT; i++)
                {
                    HAL_ADC_PollForConversion(&hadc3, 10);
                    samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc3);
                    HAL_ADC_PollForConversion(&hadc3, 10);
                    samples_dc[i] = (uint16_t)HAL_ADC_GetValue(&hadc3);
                }
                for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        if(samples[j] > samples[j+1]) { uint16_t t = samples[j]; samples[j] = samples[j+1]; samples[j+1] = t; }
                for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        if(samples_dc[j] > samples_dc[j+1]) { uint16_t t = samples_dc[j]; samples_dc[j] = samples_dc[j+1]; samples_dc[j+1] = t; }
                uint32_t sum3 = 0, sum_dc = 0;
                for(int i = 10; i < STABLE_SAMPLE_CNT - 10; i++)
                { sum3 += samples[i]; sum_dc += samples_dc[i]; }
                u_load = (uint16_t)(sum3 / 30);
                adc3_valDC = (uint16_t)(sum_dc / 30);
                #undef STABLE_SAMPLE_CNT
            }
            adc3_val = u_load;
            adc3_val1 = (uint16_t)adc3_val;

            {
                const float R_series = 4200.0f;
                if(adc1_val > adc2_val && adc2_val > 20)
                { float r = (float)adc2_val / (float)(adc1_val - adc2_val); input_resistance = r * R_series; }
                else input_resistance = 0;
            }
            mstep = 1;
        }
        else if(mstep == 1)
        {
            HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
            osDelay(100);
            osDelay(10);

            {
                #define STABLE_SAMPLE_CNT 50
                uint16_t samples[STABLE_SAMPLE_CNT];
                uint16_t samples_dc[STABLE_SAMPLE_CNT];
                HAL_ADC_Start(&hadc3);
                for(int i = 0; i < STABLE_SAMPLE_CNT; i++)
                {
                    HAL_ADC_PollForConversion(&hadc3, 10);
                    samples[i] = (uint16_t)HAL_ADC_GetValue(&hadc3);
                    HAL_ADC_PollForConversion(&hadc3, 10);
                    samples_dc[i] = (uint16_t)HAL_ADC_GetValue(&hadc3);
                }
                for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        if(samples[j] > samples[j+1]) { uint16_t t = samples[j]; samples[j] = samples[j+1]; samples[j+1] = t; }
                for(int i = 0; i < STABLE_SAMPLE_CNT - 1; i++)
                    for(int j = 0; j < STABLE_SAMPLE_CNT - 1 - i; j++)
                        if(samples_dc[j] > samples_dc[j+1]) { uint16_t t = samples_dc[j]; samples_dc[j] = samples_dc[j+1]; samples_dc[j+1] = t; }
                uint32_t sum3 = 0, sum_dc = 0;
                for(int i = 10; i < STABLE_SAMPLE_CNT - 10; i++)
                { sum3 += samples[i]; sum_dc += samples_dc[i]; }
                uo_open_value = (uint16_t)(sum3 / 30);
                adc3_valDC = (uint16_t)(sum_dc / 30);
                #undef STABLE_SAMPLE_CNT
            }
            adc3_val2 = (uint16_t)uo_open_value;

            if(adc2_val > 50 && uo_open_value > 20)
                amplification = (float)uo_open_value * 36 / (float)adc2_val;
            else
                amplification = 0;

            if(u_load > 30 && uo_open_value > u_load)
            { float ratio = (float)uo_open_value / (float)u_load; output_resistance = (ratio - 1.0f) * RELAY_LOAD_R; if(output_resistance < 0) output_resistance = 0; }
            else output_resistance = 0;
            mstep = 2;
        }
        else
        {
            Page0_Update();
            mstep = 0;
            osDelay(100);
        }
    }
    else if(current_page == 0 && !measure_running)
    {
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
        osDelay(200);
    }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

