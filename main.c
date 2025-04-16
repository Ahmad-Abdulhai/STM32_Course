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

int main() {

	/*uint8_t: Unsigned 8-bit integer (0 to 255)*/
	uint8_t age = 25;
	printf("uint8_t - Age: %u\n", age);

	/*uint16_t: Unsigned 16-bit integer (0 to 65,535)*/
	uint16_t distance = 5000;
	printf("uint16_t - Distance: %u meters\n", distance);

	/*uint32_t: Unsigned 32-bit integer (0 to 4,294,967,295)*/
	uint32_t population = 1500000000;
	printf("uint32_t - Population: %u\n", population);

	/* int: Signed 32-bit integer (-2,147,483,648 to 2,147,483,647)  --> the same int32_t*/
	int temperature = -15;
	printf("int - Temperature: %d°C\n", temperature);

	/*int8_t Signed 8-bit integer (-128 to 127 )*/
	int8_t size = 100 ;
	printf("int8_t - size : %d m^3\n", size);

	/*int16_t Signed 16-bit integer (-32,768 to +32,768 )*/
	int16_t int16Var = -32768 ;
	printf("int16_t - int16Var : %d \n", int16Var);

    return 0;
}
