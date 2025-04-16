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

int main() {

	   /*Arithmetic Operators*/
//     int a = 10, b = 3;
//	    printf("Arithmetic Operators:\n");
//	    printf("Addition: %d + %d = %d\n", a, b, a + b);           // Addition
//	    printf("Subtraction: %d - %d = %d\n", a, b, a - b);        // Subtraction
//	    printf("Multiplication: %d * %d = %d\n", a, b, a * b);     // Multiplication
//	    printf("Division: %d / %d = %d\n", a, b, a / b);           // Division
//	    printf("Modulus: %d %% %d = %d\n", a, b, a % b);         // Modulus
//	    float f = (float)a/b;
//	    printf("Division after casting: %d / %d = %f\n\n", a, b, f); // Division with casting

/******************************************************************************/
	   /*Relational Operators*/
//	   printf("Relational Operators:\n");
//	   printf("Equal: %d == %d = %d\n", a, b, a == b);            // Equal to
//	   printf("Not Equal: %d != %d = %d\n", a, b, a != b);        // Not equal to
//	   printf("Greater Than: %d > %d = %d\n", a, b, a > b);       // Greater than
//	   printf("Less Than: %d < %d = %d\n", a, b, a < b);          // Less than
//	   printf("Greater or Equal: %d >= %d = %d\n", a, b, a >= b); // Greater than or equal to
//	   printf("Less or Equal: %d <= %d = %d\n\n", a, b, a <= b); // Less than or equal to

/******************************************************************************/
	/*Logical Operators*/
//	   bool x = 1, y = 0;
//	   printf("Logical Operators:\n");
//	   printf("Logical AND: %d && %d = %d\n", x, y, x && y);      // Logical AND
//	   printf("Logical OR: %d || %d = %d\n", x, y, x || y);       // Logical OR
//	   printf("Logical NOT: !%d = %d\n\n", x, !x);                // Logical NOT

/******************************************************************************/

	   /* Assignment Operators*/
	   int c = 10;
	   printf("Assignment Operators:\n");
	   c += 5;  // Equivalent to c = c + 5
	   printf("Addition Assignment: c += 5 -> c = %d\n", c);

	   c -= 3;  // Equivalent to c = c - 3
	   printf("Subtraction Assignment: c -= 3 -> c = %d\n", c);

	   c *= 2;  // Equivalent to c = c * 2
	   printf("Multiplication Assignment: c *= 2 -> c = %d\n", c);

	   c /= 4;  // Equivalent to c = c / 4
	   printf("Division Assignment: c /= 4 -> c = %d\n", c);

	   c %= 3;  // Equivalent to c = c % 3
	   printf("Modulus Assignment: c %%= 3 -> c = %d\n\n", c);

/******************************************************************************/
	   /*Increment and Decrement Operators*/
	   int d = 5;
	   printf("Increment and Decrement Operators:\n");
	   printf("Original Value: %d\n", d);

	   d++;  // Increment
	   printf("After Increment: %d\n", d);

	   d--;  // Decrement
	   printf("After Decrement: %d\n\n", d);

	return 0;
}
