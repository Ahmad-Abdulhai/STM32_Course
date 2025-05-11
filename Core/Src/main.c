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
#include "mpu6050.h"
#include "fonts.h"
#include "horse_anim.h"
#include "stdio.h"
#include "math.h"
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
#define SCREEN_WIDTH  128  // OLED width
#define SCREEN_HEIGHT 64   // OLED height
#define CIRCLE_RADIUS 10    // Circle size
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
MPU6050_t mpu;
/*Kalman filter instances for X and Y*/
Kalman_t kalmanX, kalmanY;
/*Circle position*/
float x_pos = SCREEN_WIDTH / 2;
float y_pos = SCREEN_HEIGHT / 2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void DrawMovingCircle(void);
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
	/*Initialise the Mpu6050
	 * returned value: 1---> NotOK , 0---> OK*/
	MPU6050_Init(&hi2c1);
	/*Initialise the Mpu6050
	 * returned value: 1---> NotOK , 0---> OK*/
	MPU6050_Init(&hi2c1);
	/*Initialize Kalman filter parameters*/
	kalmanX.angle = 0;
	kalmanX.bias = 0;
	kalmanX.P[0][0] = 1;
	kalmanX.P[0][1] = 0;
	kalmanX.P[1][0] = 0;
	kalmanX.P[1][1] = 1;
	kalmanX.Q_angle = 0.001;
	kalmanX.Q_bias = 0.003;
	kalmanX.R_measure = 0.03;

	kalmanY.angle = 0;
	kalmanY.bias = 0;
	kalmanY.P[0][0] = 1;
	kalmanY.P[0][1] = 0;
	kalmanY.P[1][0] = 0;
	kalmanY.P[1][1] = 1;
	kalmanY.Q_angle = 0.001;
	kalmanY.Q_bias = 0.003;
	kalmanY.R_measure = 0.03;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*Update the circle position*/
		DrawMovingCircle();
		/*Small delay for smooth animation*/
		HAL_Delay(5);
	}

	/* USER CODE END 3 */
}
/**
 * @brief Reads MPU6050 values and moves the circle on the OLED screen.
 */
void DrawMovingCircle(void) {
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();
	double dt = (current_time - last_time) / 1000.0; // Convert ms to seconds
	last_time = current_time;
	/*Read IMU data*/
	MPU6050_Read_All(&hi2c1, &mpu);

	/*Compute tilt angles using accelerometer data*/
	double accelAngleX = atan2(mpu.Ay, mpu.Az) * 180 / M_PI;
	double accelAngleY = atan2(-mpu.Ax, sqrt(mpu.Ay * mpu.Ay + mpu.Az * mpu.Az))
			* 180 / M_PI;

	/*Apply Kalman filter for smooth angle estimation*/
	double filteredAngleX = Kalman_getAngle(&kalmanX, accelAngleX, mpu.Gx, dt);
	double filteredAngleY = Kalman_getAngle(&kalmanY, accelAngleY, mpu.Gy, dt);
	/*FIX: Invert Y-axis direction by multiplying by -1*/
	filteredAngleY *= -1;
	/*Map angles to screen coordinates (scaling factor adjusted for smooth movement)*/
	x_pos = SCREEN_WIDTH / 2 + (filteredAngleX * 1.5f);
	y_pos = SCREEN_HEIGHT / 2 + (filteredAngleY * 1.5f);

	/*Ensure circle stays within OLED bounds*/
	if (x_pos < CIRCLE_RADIUS)
		x_pos = CIRCLE_RADIUS;
	if (x_pos > SCREEN_WIDTH - CIRCLE_RADIUS)
		x_pos = SCREEN_WIDTH - CIRCLE_RADIUS;
	if (y_pos < CIRCLE_RADIUS)
		y_pos = CIRCLE_RADIUS;
	if (y_pos > SCREEN_HEIGHT - CIRCLE_RADIUS)
		y_pos = SCREEN_HEIGHT - CIRCLE_RADIUS;

	// Clear screen and draw updated circle
	SSD1306_Clear();
	SSD1306_DrawFilledCircle((int) x_pos, (int) y_pos, CIRCLE_RADIUS,
			SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
	hi2c1.Init.Timing = 0x0000020B;
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
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
