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

/* Enumeration in C, also known as enum, is a user-defined data type
 * that consists of integral constants. It is used to assign names to
 * integral constants, making the code more readable and maintainable.
 * */

/* Define an enumeration named Weekday*/
typedef enum {
	MONDAY,    // 0
	TUESDAY,   // 1
	WEDNESDAY, // 2
	THURSDAY,  // 3
	FRIDAY,    // 4
	SATURDAY,  // 5
	SUNDAY,     // 6
	InvalidDay = 255,
} Weekday;
;
int main() {

	/*Declare and initialize an enumeration variable*/
	Weekday today = WEDNESDAY;

	/* Print the value of the enumeration variable*/
	printf("Value of today: %d\n", today);

	/* Use switch-case with enumeration*/
	switch (today) {
	case MONDAY:
		printf("Today is Monday.\n");
		break;
	case TUESDAY:
		printf("Today is Tuesday.\n");
		break;
	case WEDNESDAY:
		printf("Today is Wednesday.\n");
		break;
	case THURSDAY:
		printf("Today is Thursday.\n");
		break;
	case FRIDAY:
		printf("Today is Friday.\n");
		break;
	case SATURDAY:
		printf("Today is Saturday.\n");
		break;
	case SUNDAY:
		printf("Today is Sunday.\n");
		break;
	default:
		printf("Invalid day.\n");
		break;
	}
	return 0;
}

