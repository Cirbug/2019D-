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
#include "cmsis_os.h"
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
// 扫频分步状态（跨文件引用，不能static）
uint8_t sweep_step = 0;              // 0=空闲, 1=粗扫, 2=细扫, 3=画图
int sweep_i = 0;                     // 当前循环索引
uint16_t coarse_val[20];
uint16_t coarse_max = 0;
uint8_t  coarse_max_idx = 0;
uint32_t freq_in_max = 0;
uint16_t fine_val[400];
uint16_t value_3db = 0;
uint16_t draw_val[280];
uint32_t FH = 0;
uint16_t draw_val_norm_prev = 0;     // 边扫边画时保存上一个归一化值
float av_max = 0.0f;                 // 通带最大放大倍数 Av_max = Uo_max / Ui_ref
float ui_ref = 1.0f;                 // 输入电压参考值（在粗扫最大值频率点测得）
uint16_t y_mid1 = 0;                 // Y轴1/3刻度线Y坐标
uint16_t y_mid2 = 0;                 // Y轴2/3刻度线Y坐标

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

// ======================= 界面2：故障检测 =======================
volatile uint8_t fault_running = 0;
volatile uint8_t fault_status[4] = {0,0,0,0};

// ======================= 界面3：DDS手动频率控制 =======================
uint32_t page3_freq = 1000;                // 当前DDS频率，默认1kHz
uint16_t page3_adc3_ac = 0;                // ADC3 IN4 交流值
uint16_t page3_adc3_dc = 0;                // ADC3 IN5 AD637直流值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
#include "page0.h"
#include "page1.h"
#include "page2.h"
#include "page3.h"
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
    // 初始化已迁移到 freertos.c 的 StartDefaultTask() 中
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 主循环逻辑已迁移到 freertos.c 的 StartDefaultTask() 中
    // osKernelStart() 后永远不会执行到这里
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
