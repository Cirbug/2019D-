#include "filtering.h"
#include "adc.h"
#include "stdlib.h"
//一阶互补滤波
int firstOrderFilter(int newValue, int oldValue, float a)
{
	return a * newValue + (1-a) * oldValue;
}

//中值滤波算法
int middleValueFilter(int N)
{
    int value_buf[N];
    int i,j,k,temp;
    for( i = 0; i < N; ++i)
    {
        value_buf[i] = HAL_ADC_GetValue(&hadc1);					
    }
    for(j = 0 ; j < N-1; ++j)
    {
        for(k = 0; k < N-j-1; ++k)
        {
            //从小到大排序，冒泡算法
            if(value_buf[k] > value_buf[k+1])
            {
                temp = value_buf[k];
                value_buf[k] = value_buf[k+1];
                value_buf[k+1] = temp;
            }
        }
    }
		
    return value_buf[(N-1)/2];
}

//算术均值滤波算法
int averageFilter(int N)
{
   int sum = 0;
   short i;
   for(i = 0; i < N; ++i)
   {
        sum += HAL_ADC_GetValue(&hadc1);	
   }
   return sum/N;
}

//平滑均值滤波
#define N 10
int value_buf[N];
int sum=0;
int curNum=0;
int moveAverageFilter(void)
{
    if(curNum < N)
    {
        value_buf[curNum] = HAL_ADC_GetValue(&hadc1);
        sum += value_buf[curNum];
			  curNum++;
        return sum/curNum;
    }
    else
    {
        sum -= sum/N;
        sum += HAL_ADC_GetValue(&hadc1);
        return sum/N;
    }
}

//限幅平均滤波
#define M 4    // 滑动窗口大小（可修改）
#define A 4095   // 限幅阈值设为ADC满量程，实际禁用限幅
#define ADC_NUM 3  // 固定为3个ADC (ADC1, ADC2, ADC3)

// 为每个ADC单独维护数据缓冲区、索引、初始化标志
static int data_buf[ADC_NUM][M];
static uint8_t index_buf[ADC_NUM] = {0};
static uint8_t first_flag[ADC_NUM] = {0};

/**
 * @brief  限幅滑动平均滤波（支持多ADC）
 * @param  hadc: 要采样的ADC句柄（&hadc1 / &hadc2 / &hadc3）
 * @retval 滤波后的平均值
 */
int LAverageFilter(ADC_HandleTypeDef *hadc)
{
    // 1. 自动判断是哪个ADC，分配对应的缓冲区
    uint8_t adc_id;
    if (hadc->Instance == ADC1) {
        adc_id = 0;
    } else if (hadc->Instance == ADC2) {
        adc_id = 1;
    } else if (hadc->Instance == ADC3) {
        adc_id = 2;
    } else {
        return 0; // 不支持的ADC，直接返回0
    }

    // 启动ADC转换
    HAL_ADC_Start(hadc);
    HAL_Delay(1);
    int temp = (int)HAL_ADC_GetValue(hadc);
    int sum = 0;

    // 2. 第一次采样：初始化整个缓冲区，避免数组越界
    if (first_flag[adc_id] == 0) {
        for (int i = 0; i < M; i++) {
            data_buf[adc_id][i] = temp;
        }
        first_flag[adc_id] = 1;
        index_buf[adc_id] = 0;
    } else {
        // 3. 限幅判断：超过阈值则不更新当前值，直接返回历史平均
        int last_idx = (index_buf[adc_id] == 0) ? (M-1) : (index_buf[adc_id] - 1);
        if (abs(temp - data_buf[adc_id][last_idx]) > A) {
            for (int i = 0; i < M; i++) {
                sum += data_buf[adc_id][i];
            }
            return sum / M;
        }
    }

    // 4. 更新对应ADC的缓冲区
    data_buf[adc_id][index_buf[adc_id]] = temp;
    index_buf[adc_id]++;
    if (index_buf[adc_id] >= M) {
        index_buf[adc_id] = 0;
    }

    // 5. 计算并返回平均值
    for (int i = 0; i < M; i++) {
        sum += data_buf[adc_id][i];
    }
    return sum / M;
}

/**
 * @brief  限幅滤波（阈值20）
 *         每个ADC独立维护上一次有效值
 *         若当前值与上次有效值之差超过阈值，则丢弃当前值，返回上次有效值
 * @param  newValue: 当前ADC采样值
 * @param  adcId: ADC编号（0=ADC1, 1=ADC2, 2=ADC3）
 * @retval 滤波后的值
 */
#define LIMIT_THRESHOLD 50

static int last_valid[3] = {0};
static uint8_t lim_init[3] = {0};

int LimitingFilter(int newValue, uint8_t adcId)
{
    if (adcId > 2) return newValue;
    
    if (!lim_init[adcId]) {
        last_valid[adcId] = newValue;
        lim_init[adcId] = 1;
        return newValue;
    }
    
    if (abs(newValue - last_valid[adcId]) <= LIMIT_THRESHOLD) {
        last_valid[adcId] = newValue;
    }
    
    return last_valid[adcId];
}

//卡尔曼滤波（支持多ADC）
#define KALMAN_ADC_NUM 3
typedef struct {
    float prevData;
    float p;
    float q;
    float r;
    float kGain;
    uint8_t initialized;
} KalmanState;

static KalmanState kalman_states[KALMAN_ADC_NUM] = {0};

/**
 * @brief  卡尔曼滤波（支持多ADC）
 * @param  hadc: ADC句柄（&hadc1 / &hadc2 / &hadc3）
 * @param  inData: 本次ADC采样值
 * @retval 滤波后的值
 */
int KalmanFilter(ADC_HandleTypeDef *hadc, int inData)
{
    // 判断是哪个ADC
    uint8_t adc_id;
    if (hadc->Instance == ADC1) {
        adc_id = 0;
    } else if (hadc->Instance == ADC2) {
        adc_id = 1;
    } else if (hadc->Instance == ADC3) {
        adc_id = 2;
    } else {
        return inData;
    }

    KalmanState *ks = &kalman_states[adc_id];

    // 首次初始化
    if (!ks->initialized) {
        ks->prevData = (float)inData;
        ks->p = 10.0f;
        ks->q = 0.001f;    // 过程噪声协方差（控制对变化的敏感度）
        ks->r = 0.001f;    // 测量噪声协方差（控制平滑程度）
        ks->kGain = 0.0f;
        ks->initialized = 1;
        return inData;
    }

    // 卡尔曼滤波递推
    ks->p = ks->p + ks->q;
    ks->kGain = ks->p / (ks->p + ks->r);
    ks->prevData = ks->prevData + (ks->kGain * ((float)inData - ks->prevData));
    ks->p = (1.0f - ks->kGain) * ks->p;

    return (int)ks->prevData;
}
