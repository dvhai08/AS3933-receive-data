/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS3933.h"
#include "stdbool.h"
#include "soft_timer.h"
#include "macro.h"	
#include "edge_detection.h"
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

/* USER CODE BEGIN PV */
volatile bool o_wake = 0;

bool o_dat_pin = false;
bool o_cl_dat_pin = false;
uint8_t i = 0;
uint8_t j = 0;
uint8_t b_rssi = 0;
uint8_t test_result[10] = { 0 };

uint8_t b_received_data = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void dwt_enable(void);
static uint8_t reverse_byte(uint8_t byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	dwt_enable();
	HAL_Delay(5000);

	as3933_write_Rx(AS3933_R0, /*AS3933_ON_OFF |*/AS3933_EN_A3 | AS3933_DAT_MASK);
	as3933_write_Rx(AS3933_R1, AS3933_AGC_UD | AS3933_EN_XTAL | AS3933_EN_MANCH | AS3933_EN_WPAT);
	as3933_write_Rx(AS3933_R3, AS3933_SET_FS_ENV(2) | AS3933_SET_FS_SCL(5));
	as3933_write_Rx(AS3933_R4, /*AS3933_SET_T_OFF(0x03) |*/ AS3933_SET_D_RES(0x00) | AS3933_SET_GR(0x00));
	as3933_write_Rx(AS3933_R7, AS3933_SET_T_HBIT(10));
	as3933_write_Rx(AS3933_R19, AS3933_CAP_CH3_8pF);

	as3933_reset_rssi();
	as3933_set_listening_mode();

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
		if (o_wake)
		{
			o_wake = false;
			i = 0;
			TON(0, 0, 0, 0);
			b_received_data = 0;
			
			b_rssi = as3933_read_Rx(AS3933_R12);
			while(i < 8)
			{
				o_dat_pin    = LL_GPIO_IsInputPinSet(DAT_GPIO_Port, DAT_Pin);
				o_cl_dat_pin = LL_GPIO_IsInputPinSet(CL_DAT_GPIO_Port, CL_DAT_Pin);
				
				if (edge_detection(0, o_cl_dat_pin))
				{
					o_dat_pin?SET_BIT_POS(b_received_data, i++):RESET_BIT_POS(b_received_data, i++);
				}
				
				if(TON(0, 1, HAL_GetTick(), 2000))
				{
					TON(0, 0, 0, 0);
					break;
				}
			}
			
			test_result[j++] = reverse_byte(b_received_data);

			if(j == 10) j = 0;
			
			as3933_set_listening_mode();
		}
			
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_Enable64bitAccess();

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void callback_wake(void)
{
	o_wake = true;
}

static uint8_t reverse_byte(uint8_t byte)
{
	uint8_t b_tmp = 0;

	for(uint8_t i = 0; i < 8; ++i)
	{
	  if(READ_BIT_POS(byte, i))
	  {
		  SET_BIT_POS(b_tmp, 7 - i);
	  }
	  else
	  {
		  RESET_BIT_POS(b_tmp, 7 - i);
	  }
	}
	return b_tmp;
}

void dwt_enable(void)
{
	//24.Bit TRCENA in Debug Exception and Monitor Control Register must be set before enable DWT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// CYCCNT is a free running counter, counting upwards.32 bits. 2^32 / cpuFreq = max time
	DWT->CYCCNT = 0u; // initialize the counter to 0
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the CYCCNT counter.
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

#ifdef  USE_FULL_ASSERT
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
