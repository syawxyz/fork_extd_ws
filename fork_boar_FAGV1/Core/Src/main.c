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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "docking_pallet.h"
#include "can_handle.h"
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

limit_switch ls;
ws2812_handleTypeDef led_indicator;
HAL_StatusTypeDef can_init_status;  /* hasil can_handle_init, cek di debugger jika CAN diam */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	ls.lim_port [0] = GPIOB;
	ls.lim_port [1] = GPIOA;
	ls.lim_pin [0] = GPIO_PIN_3;
	ls.lim_pin [1] = GPIO_PIN_15;

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */


	ws2812_init(&led_indicator, &htim1, TIM_CHANNEL_1, 8);
	zeroLedValues(&led_indicator);

	HAL_Delay(100);

	can_init_status = can_handle_init();

	while (1)
	{
		volatile uint32_t now;
		volatile uint32_t prev_now;
		now = HAL_GetTick();
		if (now - prev_now >= 1000)
		{
			dock_transmit.tick++;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			prev_now = now;
		}

		can_handle_send_telemetry(&dock_transmit);

	}



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//		volatile uint32_t now;
		//		volatile uint32_t prev_now;
		ls.docking_state = limit_sw_routine(&ls);
		can_handle_get_command(&dock_receive);
		uint8_t cmd_timeout = can_handle_cmd_vel_timed_out(&dock_receive); /* 1 jika >200 ms tanpa CMD_VEL */
//		now = HAL_GetTick();
		if (cmd_timeout >=100)
		{
//			now = 0;
			dock_receive.mode = 0;
			for (int i = 0; i < led_indicator.leds; i++)
			{
				setLedValues(&led_indicator, i,30, 0, 0);
			}

		}

		if (dock_receive.mode == 0b0001) // procces
		{
			if (ls.docking_state == 0b0000)
			{
				for (int i = 0; i < led_indicator.leds; i++)
				{
					setLedValues(&led_indicator, i,10, 20, 0);
				}
			}
			else if (ls.docking_state == 0b0001 || ls.docking_state == 0b0010)
			{
				for (int i = 0; i < led_indicator.leds; i++)
				{
					setLedValues(&led_indicator, i,30, 0, 0);
				}
			}

			if (ls.docking_state == 0b0011)
			{
				for (int i = 0; i < led_indicator.leds; i++)
				{
					setLedValues(&led_indicator, i,0, 30, 0);
				}
			}
		}
		else if (dock_receive.mode == 0b0010) // process fail
		{
			if (ls.docking_state == 0b0001 || ls.docking_state == 0b0010)
			{
				for (int i = 0; i < led_indicator.leds; i++)
				{
					setLedValues(&led_indicator, i,30, 0, 0);
				}
			}
		}

		else if (dock_receive.mode == 0b0011) // process succes
		{
			if (ls.docking_state == 0b0001 || ls.docking_state == 0b0010)
			{
				for (int i = 0; i < led_indicator.leds; i++)
				{
					setLedValues(&led_indicator, i,0, 30, 0);
				}
			}
		}

		dock_transmit.height = 0.0f; //TODO
		dock_transmit.state = ls.docking_state;


//		if (now - prev_now >= 1000)
		{
			dock_transmit.tick++;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			prev_now = now;
		}

		can_handle_send_telemetry(&dock_transmit);
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
