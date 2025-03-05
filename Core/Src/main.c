/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "horse_anim.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*SSD1306_HEIGHT*/
#define OLED_HEIGHT (uint8_t) 32
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void GradualFillScreen(void);
void HorseAnimation(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	 /*Initialise the display*/
	SSD1306_Init();
	/*goto 10, 10 coordinate*/
	SSD1306_GotoXY(0, 10);
	/*print Hello*/
	SSD1306_Puts("HELLO STM32", &Font_11x18, 1);
	SSD1306_UpdateScreen(); // update screen
	HAL_Delay(2000);
	/*Run the effect*/
	GradualFillScreen();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/*Run horse animation*/
		HorseAnimation();
	}
	/* USER CODE END 3 */
}
/**
 * @brief Gradually fills the OLED screen with different patterns.
 *
 * This function creates a gradual fill effect on the SSD1306 OLED display by:
 * 1. Drawing horizontal lines from top to bottom.
 * 2. Expanding a filled rectangle from the center outward.
 * 3. Drawing a growing triangle from the bottom up.
 * 4. Expanding a filled circle from the center outward.
 */
void GradualFillScreen(void) {
	/*Clear the screen initially*/
	SSD1306_Clear();
	SSD1306_UpdateScreen();

	HAL_Delay(500);

	/*Step 1: Draw horizontal lines from top to bottom*/
	for (uint16_t y = 0; y < OLED_HEIGHT; y += 4) {
		SSD1306_DrawLine(0, y, 127, y, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		/* Small delay to see the effect*/
		HAL_Delay(50);
	}
	/* Pause for effect*/
	HAL_Delay(500);

	/*Step 2: Expanding filled rectangle from the center*/
	SSD1306_Clear(); //Clear screen to update
	for (uint16_t size = 0; size < OLED_HEIGHT ; size += 4) {
		SSD1306_DrawFilledRectangle(64 - size / 2, OLED_HEIGHT/2 - size / 2, size, size,
				SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(50);
	}

	HAL_Delay(500);

	/*Step 3: Expanding triangle from bottom*/
	SSD1306_Clear(); //Clear screen to update
	for (uint16_t height = 0; height < OLED_HEIGHT; height += 4) {
		SSD1306_DrawTriangle(64, 0, 0, height, 127, height,
				SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(50);
	}
	/* Pause for effect*/
	HAL_Delay(500);

	/*Step 4: Expanding filled circle from center*/
	SSD1306_Clear();
	for (uint16_t r = 0; r < OLED_HEIGHT/2; r += 2) {
		SSD1306_DrawFilledCircle(64, OLED_HEIGHT/2, r, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(50);
	}
	/*scroll entire screen (Page0 to Page7) right*/
	SSD1306_ScrollRight(0x00, 0x07);
	/*Delay to see the scrolling*/
	HAL_Delay(5000);
	/*Stop Scrolling*/
	SSD1306_Stopscroll();
	/*scroll entire screen (Page0 to Page7) right*/
	SSD1306_ScrollLeft(0x00, 0x07);
	/*Delay to see the scrolling*/
	HAL_Delay(5000);
	/*Stop Scrolling*/
	SSD1306_Stopscroll();
	HAL_Delay(100); // Final pause before clearing the screen
}
/**
 * @brief Displays a horse running animation on the SSD1306 OLED.
 *
 * This function sequentially displays 10 different horse bitmaps
 * (horse1 to horse10) to create a smooth running animation.
 * Each frame is displayed briefly before clearing the screen and updating it.
 *
 * Steps:
 * 1. Clears the display.
 * 2. Draws the horse frame from the bitmap array.
 * 3. Updates the OLED to show the frame.
 * 4. Repeats for all frames in sequence.
 *
 * Note: Ensure `horse1` to `horse10` bitmaps are correctly defined in the code.
 */
void HorseAnimation(void) {
	//// HORSE ANIMATION START //////

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse1, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse2, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse3, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse4, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse5, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse6, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse7, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse8, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse9, 128, 64, 1);
	SSD1306_UpdateScreen();

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, horse10, 128, 64, 1);
	SSD1306_UpdateScreen();

	//// HORSE ANIMATION ENDS //////
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x0010061A;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
