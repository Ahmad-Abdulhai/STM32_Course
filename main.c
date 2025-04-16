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

		/* 1 - Casting float to int
		 *  Gives you a floating-point number to an integer, truncating any decimal places
		 *  */
		float a = 5.75423;
		int32_t b = (int) a;
		printf("Result: %d\n\n", b);
/******************************************************************************/

		/*2 - Casting char to int*/
		char ch = 'A';
		int ascii_value = (int)ch;
		printf("ASCII Value of %c: %d\n\n", ch, ascii_value);
/******************************************************************************/
		/* 3 - Casting uint32_t to uint8_t
		 *  Gives you the least significant byte of the uint32_t value.
		 * */
		uint32_t largeValue = 0x12345678;
		uint8_t smallValue = (uint8_t)(largeValue); // smallValue will be 0x78
		printf("smallValue after casting: 0x%x\n\n", smallValue);

/******************************************************************************/
		/* 4 - Casting double to float */
		double highPrecisionValue = 123.456789012345;
		float lessPrecisionValue = (float)(highPrecisionValue); // lessPrecisionValue will be approximately 123.45679
		printf("lessPrecisionValue after casting:%f\n\n", lessPrecisionValue);

    return 0;
}
