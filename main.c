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

/* 1-Function Declaration/Prototype
 * function-type function-name ( param1-type param1, param2-type param2, … );
 *
 * 2-Function Definition
 * function-type function-name( param1-type param1, param2-type param2, … ){
 *  //code for the function
 *  }
 * */
/* Function declarations -------------- */
void greet(void);
int getNumber(void);
void printSum(int a, int b);
int multiply(int a, int b);

int main() {

	/*Calling a function with no return value and no parameters*/
	greet();

	/*Calling a function with a return value and no parameters*/
	int num = getNumber();
	printf("Number: %d\n", num);

	/*Calling a function with no return value and parameters*/
	printSum(5, 7);

	/*Calling a function with a return value and parameters*/
	int result = multiply(4, 5);
	printf("Multiplication Result: %d\n", result);
	return 0;
}
/* Function definitions ---------------- */
/* 1- No return value, no parameters */
void greet(void) {
	printf("Hello, world!\n");

}
/*****************************************/

/* 2- Return value, no parameters */
int getNumber(void) {
	return rand();
}
/*****************************************/

/* 3- No return value, parameters */
void printSum(int a, int b) {
	printf("Sum: %d\n", a + b);
}
/*****************************************/

/* 4- Return value, parameters */
int multiply(int a, int b) {
	return a * b;
}

