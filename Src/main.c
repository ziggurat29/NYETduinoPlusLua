/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <fcntl.h>

#include "usbd_cdc.h"	//just for the XXX_USBCDC_PresenceHack()

//XXX abandoning newlib approach
//#include "newlib/newlib_device.h"

#include "system_interfaces.h"
#include "serial_devices.h"



//This controls whether we use the FreeRTOS heap implementation to also provide
//the libc malloc() and friends.  Note, if you turn this off, you will want to
//adjust _Min_Heap_Size in the linker script, because it is set to a very low
//value.
#define USE_FREERTOS_HEAP_IMPL 1


#if USE_FREERTOS_HEAP_IMPL

#if configAPPLICATION_ALLOCATED_HEAP
//we define our heap (to be used by FreeRTOS heap_4.c implementation) to be
//exactly where we want it to be.
__attribute__((aligned(8))) 
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif
//we implemented a 'realloc' for a heap_4 derived implementation
extern void* pvPortRealloc( void* pvOrig, size_t xWantedSize );
//we implemented a 'heapwalk' function
typedef int (*CBK_HEAPWALK) ( void* pblk, uint32_t nBlkSize, int bIsFree );
extern int vPortHeapWalk ( CBK_HEAPWALK pfnWalk );

//'wrapped functions' for library interpositioning
//you must specify these gcc (linker-directed) options to cause the wrappers'
//delights to be generated:

//-Wl,--wrap,malloc -Wl,--wrap,free -Wl,--wrap,realloc -Wl,--wrap,calloc
//-Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_calloc_r

//hmm; can I declare these 'inline' and save a little code and stack?
void* __wrap_malloc ( size_t size ) { return pvPortMalloc ( size ); }
void __wrap_free ( void* pv ) { vPortFree ( pv ); }
void* __wrap_realloc ( void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }

void* __wrap__malloc_r ( struct _reent* r, size_t size ) { return pvPortMalloc ( size ); }
void __wrap__free_r ( struct _reent* r, void* pv ) { vPortFree ( pv ); }
void* __wrap__realloc_r ( struct _reent* r, void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }



#endif



#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;

/* USER CODE BEGIN PV */
//spi3 == sd card
//spi1 == enc28j60


/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void led_pwr_off()
{
	HAL_GPIO_WritePin(PWR_LED_OFF_GPIO_Port, PWR_LED_OFF_Pin, GPIO_PIN_RESET);
}
void led_pwr_on()
{
	HAL_GPIO_WritePin(PWR_LED_OFF_GPIO_Port, PWR_LED_OFF_Pin, GPIO_PIN_SET);
}

void led_blu_off()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
void led_blu_on()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}



int switch1_read()
{
	//pressed = high
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(SWITCH1_B_GPIO_Port, SWITCH1_B_Pin);
}



int sd_cardpresent_read()
{
	//inserted == low
	return GPIO_PIN_RESET == HAL_GPIO_ReadPin(MICRO_SD_CARD_INSERTED_GPIO_Port, MICRO_SD_CARD_INSERTED_Pin);
}

void sd_cs_active()
{	//active low
	HAL_GPIO_WritePin(MICROSD_CS_GPIO_Port, MICROSD_CS_Pin, GPIO_PIN_RESET);
}
void sd_cs_inactive()
{	//active low
	HAL_GPIO_WritePin(MICROSD_CS_GPIO_Port, MICROSD_CS_Pin, GPIO_PIN_SET);
}

void sd_pwrctl_on()
{	//active low
	HAL_GPIO_WritePin(PWR_CTRL_MICROSD_GPIO_Port, PWR_CTRL_MICROSD_Pin, GPIO_PIN_RESET);
}
void sd_pwrctl_off()
{	//active low
	HAL_GPIO_WritePin(PWR_CTRL_MICROSD_GPIO_Port, PWR_CTRL_MICROSD_Pin, GPIO_PIN_SET);
}



int eth_int_read()
{
	//interrupt == low
	return GPIO_PIN_RESET == HAL_GPIO_ReadPin(ENC_INT_GPIO_Port, ENC_INT_Pin);
}

void eth_cs_active()
{	//active low
	HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
}
void eth_cs_inactive()
{	//active low
	HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
}

void eth_reset_active()
{	//active low
	HAL_GPIO_WritePin(ENC_RESET_GPIO_Port, ENC_RESET_Pin, GPIO_PIN_RESET);
}
void eth_reset_inactive()
{	//active low
	HAL_GPIO_WritePin(ENC_RESET_GPIO_Port, ENC_RESET_Pin, GPIO_PIN_SET);
}

void eth_pwrctl_on()
{	//active low
	HAL_GPIO_WritePin(PWR_CTRL_ETHERNET_GPIO_Port, PWR_CTRL_ETHERNET_Pin, GPIO_PIN_RESET);
}
void eth_pwrctl_off()
{	//active low
	HAL_GPIO_WritePin(PWR_CTRL_ETHERNET_GPIO_Port, PWR_CTRL_ETHERNET_Pin, GPIO_PIN_SET);
}


void headers_pwrctl_on()
{	//active high
	HAL_GPIO_WritePin(CTRL_OF_PWR_HEADERS_GPIO_Port, CTRL_OF_PWR_HEADERS_Pin, GPIO_PIN_SET);
}
void headers_pwrctl_off()
{	//active high
	HAL_GPIO_WritePin(CTRL_OF_PWR_HEADERS_GPIO_Port, CTRL_OF_PWR_HEADERS_Pin, GPIO_PIN_RESET);
}



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	//enable the core debug cycle counter to be used as a precision timer
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	//XXX crap! STM32Workbench thinks it's cool to have a global date and time
	//for some reason, and furthermore does not init them in a way that will
	//not assert later on.  Rude.  Does anyone QA this stuff?
	//sTime.
	sDate.Year = 18;
	sDate.Month = 1;
	sDate.Date = 1;
	sDate.WeekDay = 1;	//(does monday=1?)
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	//if you get a linker fail on the following, it is because some manual
	//changes to:
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c
	//  .\Src\usbd_cdc_if.c
	//must be applied.  There are backups of those files to help with that.
	//This has to be done manually, because the changes are in tool-generated
	//code that gets overwritten when you re-run STM32CubeMX.  The nature of
	//those changes are such that when they are overwritten, you will still
	//be able to build but stuff won't work at runtime.  This hack will cause
	//the build to fail if you forget to merge those changes back on, thus
	//prompting you to do so.
	//Sorry for the inconvenience, but I don't think there is any better way
	//of making it obvious that this chore simply must be done.
	XXX_USBCDC_PresenceHack();	//this does nothing real; do not delete

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	//dummy alloc to cause FreeRTOS to initialize heap
	volatile uint8_t* pvDummy = (uint8_t*) malloc ( 10 );
	memset ( (uint8_t*)pvDummy, 0xa5, 10 );
	free ( (void*)pvDummy );

	// Initialize device manager
	//XXX this is static and const, but the devices themselves may need initing

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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	NVIC_SystemReset();	//not supposed to be here!
//  while (1)
//  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV25;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
//XXX what is this insufferable junk? q.v. stm32f4xx_hal_rtc.c:653; of course it will assert, dummkopf; it's explicitly set to zero! does anyone test this shit before release? because I am just not convinced.  I waste so much time with this crappy code, on top of this exquisitely slow java based IDE.  It's enough to make me go back to MicroChip PIC32 parts.  Think about it.
/*
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
*/

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO_1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_OFF_GPIO_Port, PWR_LED_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR_CTRL_ETHERNET_Pin|ENC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MICROSD_CS_Pin|PWR_CTRL_MICROSD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTRL_OF_PWR_HEADERS_GPIO_Port, CTRL_OF_PWR_HEADERS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_RESET_GPIO_Port, ENC_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PWR_LED_OFF_Pin PWR_CTRL_ETHERNET_Pin ENC_CS_Pin */
  GPIO_InitStruct.Pin = PWR_LED_OFF_Pin|PWR_CTRL_ETHERNET_Pin|ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH1_A_Pin */
  GPIO_InitStruct.Pin = SWITCH1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH1_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin 
                           A4_Pin A5_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin 
                          |A4_Pin|A5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D8_Pin D7_Pin D3_Pin D2_Pin 
                           D9_Pin */
  GPIO_InitStruct.Pin = D8_Pin|D7_Pin|D3_Pin|D2_Pin 
                          |D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INT_Pin */
  GPIO_InitStruct.Pin = ENC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MICROSD_CS_Pin PWR_CTRL_MICROSD_Pin CTRL_OF_PWR_HEADERS_Pin */
  GPIO_InitStruct.Pin = MICROSD_CS_Pin|PWR_CTRL_MICROSD_Pin|CTRL_OF_PWR_HEADERS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D10_Pin D4_Pin D13_Pin D12_Pin 
                           D11_Pin D5_Pin D6_Pin */
  GPIO_InitStruct.Pin = D10_Pin|D4_Pin|D13_Pin|D12_Pin 
                          |D11_Pin|D5_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH1_B_Pin MICRO_SD_CARD_INSERTED_Pin */
  GPIO_InitStruct.Pin = SWITCH1_B_Pin|MICRO_SD_CARD_INSERTED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLOCK_OUT_25MHZ_Pin */
  GPIO_InitStruct.Pin = CLOCK_OUT_25MHZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(CLOCK_OUT_25MHZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_RESET_Pin */
  GPIO_InitStruct.Pin = ENC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//====================================================



void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	called if a stack overflow is detected. */
	volatile int i = 0;
	(void)i;
}



void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created. It is also called by various parts of the
	demo application. If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	volatile int i = 0;
	(void)i;
}



char buffer[100];

void myprintf(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, 1000);
}




void __filesystem_test ( void )
{
	FRESULT fres;

	//Mount drive
	fres = f_mount(&USERFatFS, "", 1); //1=mount now
	if (fres != FR_OK)
	{
		myprintf("f_mount error (%i)\r\n", fres);
		return;
	}

	DWORD free_clusters, free_sectors, total_sectors;
	FATFS* getFreeFs;
	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK)
	{
		myprintf("f_getfree error (%i)\r\n", fres);
		return;
	}

	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;

	myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

	//Try to open file
	fres = f_open(&USERFile, "test.txt", FA_READ);
	if (fres != FR_OK)
	{
		myprintf("f_open error (%i)\r\n");
		return;
	}
	myprintf("I was able to open 'test.txt' for reading!\r\n");

	BYTE readBuf[30];

	//We can either use f_read OR f_gets to get data out of files
	//f_gets is a wrapper on f_read that does some string formatting for us
	TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &USERFile);
	if(rres != 0)
	{
		myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
	}
	else
	{
		myprintf("f_gets error (%i)\r\n", fres);
	}

	//Close file, don't forget this!
	f_close(&USERFile);

	fres = f_open(&USERFile, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if(fres == FR_OK)
	{
		myprintf("I was able to open 'write.txt' for writing\r\n");
	}
	else
	{
		myprintf("f_open error (%i)\r\n", fres);
	}

	strncpy((char*)readBuf, "a new file is made!", 19);
	UINT bytesWrote; 
	fres = f_write(&USERFile, readBuf, 19, &bytesWrote);
	if(fres == FR_OK)
	{
		myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
	}
	else
	{
		myprintf("f_write error (%i)\r\n");
	}

	//Close file, don't forget this!
	f_close(&USERFile);

	//De-mount drive
	f_mount(NULL, "", 0);
}


/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */

	sd_pwrctl_off();
	osDelay ( 500 );
	sd_pwrctl_on();


	__filesystem_test();

#if 0
#define FCLK_SLOW() { \
	__HAL_SPI_DISABLE(&hspi3); \
	uint16_t nReg = READ_REG(hspi3.Instance->CR1); \
	nReg &= ~ (7<<3); \
	nReg |= SPI_BAUDRATEPRESCALER_128; \
	WRITE_REG(hspi3.Instance->CR1, nReg ); \
	__HAL_SPI_ENABLE(&hspi3); \
}
#define FCLK_FAST() { \
	__HAL_SPI_DISABLE(&hspi3);	\
	uint16_t nReg = READ_REG(hspi3.Instance->CR1);	\
	nReg &= ~ (7<<3);	\
	nReg |= SPI_BAUDRATEPRESCALER_2;	\
	WRITE_REG(hspi3.Instance->CR1, nReg );	\
	__HAL_SPI_ENABLE(&hspi3);	\
}
#define CS_HIGH()	{HAL_GPIO_WritePin(MICROSD_CS_GPIO_Port, MICROSD_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(MICROSD_CS_GPIO_Port, MICROSD_CS_Pin, GPIO_PIN_RESET);}

	//FCLK_SLOW();
	FCLK_FAST();
	while ( 1 )
	{
		HAL_StatusTypeDef hret;
		CS_LOW();
		led_blu_on();
		for ( int i = 0; i < 1000; ++i )
		{
			BYTE txDat = 0xf0;
			BYTE rxDat;
			hret = HAL_SPI_TransmitReceive(&hspi3, &txDat, &rxDat, 1, 50);
		}
		CS_HIGH();
		led_blu_off();
		for ( int i = 0; i < 1000; ++i )
		{
			BYTE txDat = 0xcc;
			BYTE rxDat;
			hret = HAL_SPI_TransmitReceive(&hspi3, &txDat, &rxDat, 1, 50);
		}
	}
#endif

	//get our serial ports initialized
	UART6_Init();		//UART 6 == 'COM1'
	USBCDC_Init();		//CDC == 


	volatile UBaseType_t uxMinFreeStack;
	volatile UBaseType_t uxMaxSizeHeap;
	volatile UBaseType_t uxMaxUsedHeap;
	volatile UBaseType_t uxMinFreeHeap;

	uxMinFreeStack = uxTaskGetStackHighWaterMark( NULL );
#if USE_FREERTOS_HEAP_IMPL
	uxMaxSizeHeap = configTOTAL_HEAP_SIZE;
	uxMinFreeHeap = xPortGetMinimumEverFreeHeapSize();
#else
	uxMaxSizeHeap = (char*)platform_get_last_free_ram( 0 ) - (char*)platform_get_first_free_ram( 0 );
#endif

/*//XXX abandoning newlib approach
	//crank up the system; especially for the newlib bottom edge support
	if ( 1 )
	{
		//open ttys0, and redirect stdin/stdout to it; redirect stderr into stdout
//		int hTtyS0 = 0;
		int hTtyS0 = open ( "/ttys0/xxx", O_RDONLY );	//XXX haque, need to fix the dm lookup
		char achFName[64];
		sprintf ( achFName, "/std/%d=%d", STDIN_FILENO, hTtyS0 );	//stdin
		int hStdin = open ( achFName, O_RDONLY );
		sprintf ( achFName, "/std/%d=%d", STDOUT_FILENO, hTtyS0 );	//stdout
		int hStdout = open ( achFName, O_RDONLY );
		sprintf ( achFName, "/std/%d=%d", STDERR_FILENO, STDOUT_FILENO );	//stderr
		int hStderr = open ( achFName, O_RDONLY );
	}
*/

/*//XXX abandoning newlib approach
	if ( 1 )
	{
		char achBuff[128];
		int nEof;
		while ( ! ( nEof = feof ( stdin ) ) )
		{
			char* pchRet = fgets ( achBuff, sizeof(achBuff), stdin );
			if ( NULL != pchRet )
			{
				int nPutRet;
				nPutRet = fputs ( ">>", stdout );
				nPutRet = fputs ( achBuff, stdout );
				nPutRet = fputs ( "\n", stdout );
				fflush ( stdout );
			}
		}
	}
*/

//
{
#if 0
	lua_State* L = luaL_newstate();
	if ( NULL == L )
	{
		//horror;
	}
	else
	{
		int status, result;
		//lua_pushcfunction(L, &pmain);  /* to call 'pmain' in protected mode */
		//lua_pushinteger(L, argc);  /* 1st argument */
		//lua_pushlightuserdata(L, argv); /* 2nd argument */
		//status = lua_pcall(L, 2, 1, 0);  /* do the call */
		result = lua_toboolean(L, -1);  /* get result */
		//report(L, status);
		lua_close(L);
	}
#else
	extern int luashell_main(void);
//	luashell_main();
#endif
}

	uxMinFreeStack = uxTaskGetStackHighWaterMark( NULL );
#if USE_FREERTOS_HEAP_IMPL
	uxMinFreeHeap = xPortGetMinimumEverFreeHeapSize();
	uxMaxUsedHeap = uxMaxSizeHeap - uxMinFreeHeap;
#else
	extern char* heap_ptr;
	uxMaxUsedHeap = heap_ptr - (char*)platform_get_first_free_ram( 0 );
	uxMinFreeHeap = (char*)platform_get_last_free_ram( 0 ) - heap_ptr;
#endif

	//XXX abandoning newlib approach
	//printf ( "minfreestack: %lu words; maxheapused: %lu of %lu (minfree %lu)\n",
	//		uxMinFreeStack, uxMaxUsedHeap, uxMaxSizeHeap, uxMinFreeHeap );

	//heapwalk
	//vPortHeapWalk ( cbkHeapWalk );

	//XXX abandoning newlib approach
	//printf ( "resetting...\n" );
	osDelay ( 500 );	//delay a little to let all that go out before we reset
	NVIC_SystemReset();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	//XXX the other timer switches would go here
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	volatile int bSpin = 1;
	while(bSpin)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

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
	volatile int bSpin = 1;
	while(bSpin)
	{
	}
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
