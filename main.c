/*
 * SourceFile main.C
 * Description:
 *  Created on: April , 16, 2025
 *  Author: Ahmad Abdulhai @ Hexabitz

 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/*
 * A macro is a preprocessor directive, defined using #define,
 *  which tells the compiler to replace the macro name with its value or code before compilation.
 *
 *  There are two main types:
 *  1- Object-like macros – constants
 *  2- Function-like macros – inline code blocks with parameters
 * */
#define PI         3.14159f
#define VREF_MV    ((uint32_t)3300)
#define MAX_COUNT  100
#define MAX_BUFFER 10
#define NUM_OF_SENSOR 5
#define SQUARE(x)   ((x) * (x))
/********************************************************************************/
//   /* Object-like macros*/
//#define LED_PORT        GPIOA
//#define LED_PIN         GPIO_PIN_5
//#define PORTA_ON  GPIOA->ODR = 0Xffff
//#define PORTA_OFF GPIOA->ODR = 0X0000
//   /* Function-like macros*/
//#define LED_ON()        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
//#define LED_OFF()       HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
//#define LED_TOGGLE()    HAL_GPIO_TogglePin(LED_PORT, LED_PIN)
/********************************************************************************/
#define BIT_SET(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define BIT_CLEAR(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define BIT_FLIP(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define BIT_GET(ADDRESS,BIT) (ADDRESS & (1<<BIT))
int main() {

	uint16_t Square = SQUARE(10);
	printf("Square = %d\n", Square);
	uint8_t a = 5;
	Square = SQUARE(a + 1);
	printf("Square a with plus one = %d\n", Square);

	/********************************************************************************/
	int8_t controlRegister = 0b00000110;
	BIT_SET(controlRegister, 0);
	printf("controlRegister after set = 0x%x \n", controlRegister);
	return 0;
}

