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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <stdio.h>
	#include <string.h>
	#include "PhotoTimer_config.h"
	#include "tm1637_sm.h"
	#include "flash_stm32f103_hal_sm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define	PERIOD_QNT	4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	volatile	uint32_t  	timer_1sec_flag = 0 ;
	volatile	uint32_t	button[5] = { 0 } ;
				uint32_t 	time_cnt = 0;
				char 		DataChar[0xFF];
				uint32_t 	period[PERIOD_QNT] = { 10, 2, 15, 2} ;

	tm1637_struct htm1637;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	void RelayOn  (void);
	void RelayOff (void);
	void DisplayOff (void);

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	RelayOff();

	int soft_version_arr_int[3];
	soft_version_arr_int[0] = ((SOFT_VERSION) / 100)     ;
	soft_version_arr_int[1] = ((SOFT_VERSION) /  10) %10 ;
	soft_version_arr_int[2] = ((SOFT_VERSION)      ) %10 ;
	sprintf(DataChar,"\r\n\r\n\tPhotoTimer v%d.%d.%d \r\nUART1 for debug on speed 115200 \r\n",
			soft_version_arr_int[0] ,
			soft_version_arr_int[1] ,
			soft_version_arr_int[2] ) ;
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"Build: %s. Time: %s.\r\n" ,
			DATE_as_int_str ,
			TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;


	htm1637.clk_pin	= TM_CLK_Pin ;
	htm1637.clk_port= TM_CLK_GPIO_Port ;
	htm1637.dio_pin = TM_DIO_Pin ;
	htm1637.dio_port= TM_DIO_GPIO_Port ;
	tm1637_Init( &htm1637 );
	tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
	tm1637_Display_Decimal( &htm1637, 8888, double_dot ) ;
	HAL_Delay(500);
	DisplayOff();

	#define CX_FLASH_PAGE_ADDR ((uint32_t)0x08004000)
	 uint32_t flash_word_u32;
	 flash_word_u32 = Flash_Read(CX_FLASH_PAGE_ADDR);
	sprintf(DataChar,"flash_word_u32 = %lX\r\n",  flash_word_u32 );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	period[0] = (flash_word_u32>>16) ;
	period[2] = flash_word_u32&0xFFFF ;

	for (int i=0; i < PERIOD_QNT; i++) {
		tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
		tm1637_Display_Decimal( &htm1637, (i+1)*1000 +period[i], no_double_dot ) ;
		sprintf(DataChar,"period[%d] = %lu\r\n", i, period[i] );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(700);
		DisplayOff();
	}
	DisplayOff();
	HAL_Delay(500);

	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (int p=0; p < PERIOD_QNT; p++) {
		  if ((p==0) || (p==2)) {
			  RelayOn();
		  } else {
			  RelayOff();
		  }

		  time_cnt = 0;
		  do {
			  if (button[0] == 1) {
				  TIM3->CNT = 1;
				  	period[0]++;
					tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
					tm1637_Display_Decimal( &htm1637, 1000 + period[0], no_double_dot ) ;
					HAL_Delay(100);
					button[0] = 0 ;
			  }
			  if (button[1] == 1) {
				  TIM3->CNT = 1;
				  	period[0]--;
					tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
					tm1637_Display_Decimal( &htm1637, 1000 + period[0], no_double_dot ) ;
					HAL_Delay(100);
					button[1] = 0 ;
			  }
			  if (button[2] == 1) {
				  TIM3->CNT = 1;
					period[2]++;
					tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
					tm1637_Display_Decimal( &htm1637, 3000 + period[2], no_double_dot ) ;
					HAL_Delay(100);
					button[2] = 0 ;
			  }
			  if (button[3] == 1) {
				  TIM3->CNT = 1;
					period[2]--;
					tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
					tm1637_Display_Decimal( &htm1637, 3000 + period[2], no_double_dot ) ;
					HAL_Delay(100);
					button[3] = 0 ;
			  }
			  if (button[4] == 1) {
				  TIM3->CNT = 1;
					sprintf(DataChar,"Write to EEPROM \r\n" ) ;
					HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

					 uint32_t flash_word_u32;
					flash_word_u32 = (period[0]<<16) + period[2] ;
					 HAL_FLASH_Unlock();
					 Flash_Erase_Page(CX_FLASH_PAGE_ADDR);
					 Flash_Write( CX_FLASH_PAGE_ADDR, flash_word_u32);
					 HAL_FLASH_Lock();
						sprintf(DataChar,"flash_word_u32 = %lX\r\n",  flash_word_u32 );
						HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

					tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
					tm1637_Display_Decimal( &htm1637, 7777, no_double_dot ) ;
					HAL_Delay(200);
					button[4] = 0 ;
			  }

			  if (timer_1sec_flag == 1) {
				sprintf(DataChar,"timer%d = %lu\r\n",  p,  period[p] - time_cnt );
				HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
				tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
				tm1637_Display_Decimal( &htm1637, (p+1)*1000 + period[p] - time_cnt, no_double_dot ) ;
				time_cnt++;
				timer_1sec_flag = 0;
			  }
		  } while (time_cnt <= period[p]) ;
		  DisplayOff();
	  }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void RelayOn  (void) {
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET) ;
	sprintf(DataChar,"relay On\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
}

void RelayOff (void) {
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET) ;
	sprintf(DataChar,"relay Off\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
}

void DisplayOff (void){
	tm1637_Set_Brightness( &htm1637, bright_off ) ;
	//tm1637_Display_Decimal( &htm1637, 0, no_double_dot ) ;
	HAL_Delay(100);
//	tm1637_Set_Brightness( &htm1637, bright_45percent ) ;
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
