/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 24/12/2015 18:24:46
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
// #include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stm32l152c_discovery.h"
#include "stm32l152c_discovery_glass_lcd.h"
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <median.h>


typedef struct {
  void * buf;
  size_t size;
  uint32_t nf;
} tx_buf_t;

typedef enum {
  BT_UNDEFINED = 0,
  BT_IS_PRESENT,
  BT_PAIRED
} bt_status_e;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

// osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
static TaskHandle_t xMainHandle = NULL;
static TaskHandle_t xTestHandle = NULL;
static TaskHandle_t xSensorTuneHandle = NULL;
static TaskHandle_t xUsartTXHandle = NULL;
static TaskHandle_t xUsartRXHandle = NULL;

static QueueHandle_t xUartTxQueue = NULL;
static QueueHandle_t xUartRxQueue = NULL;

#define NUM_ELEMENTS(x) (sizeof(x)/sizeof(x[0]))
#define NUM_ADC_CHANNELS 2
#define ADC_SAMPLES_PER_FRAME 1
#define ADC_DATA_BUFFER_SIZE (NUM_ADC_CHANNELS * ADC_SAMPLES_PER_FRAME * 2)
#define ADC_GATE1 0
#define ADC_GATE2 1
#define RESET_H06() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define UNRESET_H06() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

static uint16_t adc_data[ADC_DATA_BUFFER_SIZE] __attribute__ ((aligned));
static u16_median_t * median[2];
static uint8_t uart_data;
static bt_status_e bt_status = BT_UNDEFINED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
// void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void vMainTask( void * pvParameters );
void vTestTask( void * pvParameters );
void vSensorTuneTask( void * pvParameters );
void vUartTxTask( void * pvParameters );
void vUartRxTask( void * pvParameters );
static void DAC_SetValue(uint32_t Channel, uint32_t Data);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void start_acquire(void)
{
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_data, NUM_ELEMENTS(adc_data));
  HAL_TIM_Base_Start(&htim3);
}

// static void stop_acquire(void)
// {
//   HAL_TIM_Base_Stop(&htim3);
//   HAL_ADC_Stop_DMA(&hadc);
// }


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

  // xTaskNotifyFromISR(xDSPHandle, (uint32_t)(&adc_data[NUM_ELEMENTS(adc_data)/2]), 
  //   eSetValueWithOverwrite, NULL);
  // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // xTaskNotifyFromISR(xDSPHandle, (uint32_t)(&adc_data[0]), eSetValueWithOverwrite, 
  //   NULL);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  vTaskNotifyGiveFromISR(xUsartTXHandle, NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  xQueueSendFromISR( xUartRxQueue, &uart_data, NULL);
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);
}


static void DAC_SetValue(uint32_t Channel, uint32_t Data)
{
  HAL_DAC_SetValue(&hdac, Channel, DAC_ALIGN_12B_R, Data);
}

static BaseType_t send_data(const void *addr, size_t size, uint32_t nf)
{
  tx_buf_t *b = (tx_buf_t *)pvPortMalloc(sizeof(*b));

  configASSERT( b );

  b->buf = (void *)addr;
  b->size = size;
  b->nf = nf;

  return xQueueSend( xUartTxQueue, &b, ( TickType_t ) 0 );
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  
  /* LCD GLASS Initialization */
  // BSP_LCD_GLASS_Init();
  median[0] = u16_median_init(5);
  median[1] = u16_median_init(5);

  start_acquire();

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  DAC_SetValue(DAC_CHANNEL_1, 0);
  DAC_SetValue(DAC_CHANNEL_2, 0);

  UNRESET_H06();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate( vMainTask, "MAIN", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xMainHandle );
  configASSERT( xMainHandle ); 
  xTaskCreate( vTestTask, "TEST", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xTestHandle );
  configASSERT( xTestHandle ); 
  xTaskCreate( vSensorTuneTask, "TUNE", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xSensorTuneHandle );
  configASSERT( xSensorTuneHandle ); 
  xTaskCreate( vUartTxTask, "UARTTX", configMINIMAL_STACK_SIZE, NULL, 1, &xUsartTXHandle );
  configASSERT( xUsartTXHandle );  
  xTaskCreate( vUartRxTask, "UARTRX", configMINIMAL_STACK_SIZE, NULL, 2, &xUsartRXHandle );
  configASSERT( xUsartRXHandle );  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xUartTxQueue = xQueueCreate(5, sizeof(tx_buf_t *));
  configASSERT( xUartTxQueue );  
  xUartRxQueue = xQueueCreate(10, sizeof(uint8_t));
  configASSERT( xUartRxQueue ); 
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  // osKernelStart(NULL, NULL);
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Receive_IT(&huart1, &uart_data, 1);

  

  vTaskStartScheduler();
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE();

}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 2;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_21;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

    /**DAC channel OUT2 config 
    */
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // /*Configure GPIO pins : PC0 PC1 PC2 PC3 
  //                          PC6 PC7 PC8 PC9 
  //                          PC10 PC11 */
  // GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
  //                         |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
  //                         |GPIO_PIN_10|GPIO_PIN_11;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  // GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // /*Configure GPIO pins : PA1 PA2 PA3 PA8 
  //                          PA9 PA10 PA15 */
  // GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8 
  //                         |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  // GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // /*Configure GPIO pins : PB10 PB11 PB12 PB14 
  //                          PB3 PB4 PB5 PB8 
  //                          PB9 */
  // GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14 
  //                         |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8 
  //                         |GPIO_PIN_9;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  // GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void vMainTask( void * pvParameters )
{
  // uint8_t str[7];
  // uint8_t cnt = 0;
  for(;;)
  {
    // snprintf((char *)str, sizeof(str)-1, "%d", cnt++);
    // BSP_LCD_GLASS_Clear();
    // BSP_LCD_GLASS_DisplayString((uint8_t *)"888888");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTestTask( void * pvParameters )
{
  // uint8_t dac = 0;
  
  for(;;)
  {
    // DAC_SetValue(DAC_CHANNEL_1, dac);
    // DAC_SetValue(DAC_CHANNEL_2, dac++);    

    send_data("AT", strlen("AT"), 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

void vSensorTuneTask( void * pvParameters )
{
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vUartTxTask( void * pvParameters )
{
  for( ;; )
  {
    tx_buf_t * txb = NULL;
    xQueueReceive(xUartTxQueue, &txb, portMAX_DELAY);
    if(txb)
    {
      HAL_StatusTypeDef st __attribute__((unused)) = 
        HAL_UART_Transmit_DMA(&huart1, txb->buf, txb->size);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      if(txb->nf && txb->buf)
        vPortFree(txb->buf);
      vPortFree(txb);
    }
  }
}

void vUartRxTask( void * pvParameters )
{
  static uint8_t rx_buf[255], pos = 0;
  for( ;; )
  {
    uint8_t data;
    BaseType_t status = xQueueReceive(xUartRxQueue, &data, portMAX_DELAY /*pdMS_TO_TICKS(10)*/);

    rx_buf[pos] = data;
    if(++pos > sizeof(rx_buf)-1)
      pos = 0;
  }
}
/* USER CODE END 4 */

/* StartDefaultTask function */
// void StartDefaultTask(void const * argument)
// {

//   /* USER CODE BEGIN 5 */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END 5 */ 
// }

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
