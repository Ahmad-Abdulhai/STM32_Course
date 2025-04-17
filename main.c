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
#include <stdlib.h>
#include <stdint.h>
/*
 * In C, parameters can be passed by value, where a copy of the data is
 * made, or by reference, using pointers, which allows the function to modify
 * the original data.
 * there is three case to passing the arguments to function
 * 1- Pass by value
 * 2- Pass by reference
 * 3- Pass by array
 */
/* Function declarations -------------- */
uint8_t Add_5_PassByValue(uint8_t value);
void swap(int *firstVar, int *secondVar);
void printArray(int arr[], int size);
int main() {

	/*Calling the function and passing it parameters by Value*/
	uint8_t result = Add_5_PassByValue(10);
	printf("Result Passing by value = %d\n\n", result);

	/******************************************************************************/
	/*Calling the function and passing it parameters by Reference*/
	int a = 10, b = 20;
	swap(&a, &b);
	printf("a: %d, b: %d\n\n", a, b);

	/******************************************************************************/
	int arr[5] = { 1, 2, 3, 4, 5 };
	/*Calling function and Passing array to the print the elements of array*/
	printArray(arr, 5);

	return 0;
}
/* Function definitions ---------------- */

uint8_t Add_5_PassByValue(uint8_t value) {
	value += 5;
	return value;
}
/*****************************************/
void swap(int *firstVar, int *secondVar) {
	int temp = *firstVar;
	*firstVar = *secondVar;
	*secondVar = temp;
}
/*****************************************/
void printArray(int arr[], int size) {
	printf("Array elements:\n");
	for (int i = 0; i < size; i++) {
		printf("%d ", arr[i]);
	}
}
