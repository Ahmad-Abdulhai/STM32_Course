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

	/* Description: An array is a collection of elements
	 * of the same type stored in contiguous memory locations.
	 *
	 * Arrays are used in embedded systems for handling multiple similar data
	 * efficiently, such as sensor readings, buffer storage, and lookup tables.
	 *
	 *  Array Initialization Syntax:
	 * type arrayName[arraySize] = {val1, val2, ..., valN};
	 * type arrayName[arraySize] = {0}; // Initialize all to 0
	 **/

	/* Declare and initialize an array of student scores*/
	int scores[5] = { 78, 85, 92, 67, 88 };
	int threshold = 90;
	int i;

	/*Check if any score is above the threshold*/
	for (i = 0; i < 5; i++) {
		if (scores[i] > threshold) {
			printf("Score above threshold: scores[%d] = %d\n", i, scores[i]);
		}
	}

/******************************************************************************/
	/* Strings in C: A Special Kind of Array.
	 * Strings are arrays of characters terminated by a null character \0.
	 *
	 * Strings are often used for storing data read from or to be written to
     *  peripherals, like displays in embedded systems
	 */

	/*Declare and initialize an string messages*/
	char errorMessage[20] = "Error Code: ";
	char str[] = "\nHello, World!"; //NOTE! String initialization automatically includes the null terminator.
	printf(errorMessage);
	printf(str);
	return 0;
}
