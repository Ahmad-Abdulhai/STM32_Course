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

	int score = 80;

	// Input the score
	printf("your score: %d\n\n", score);

	// Using if-else statement to determine the grade
	printf("Using if-else statement to determine the grade: \n");
//	if (score >= 0) {
//		printf("Grade is positive \n");
//	}
	if ((score >= 90)) {
		printf("Grade: A\n\n");
	} else if (score >= 80) {
		printf("Grade: B\n\n");
	} else if (score >= 70) {
		printf("Grade: C\n\n");
	} else if (score >= 60) {
		printf("Grade: D\n\n");
	} else {
		printf("Grade: F\n\n");
	}

	// Using switch statement to display a message based on the grade
	printf(
			"Using switch statement to display a message based on the grade: \n");
	switch (score / 10) {
	case 10:
	case 9:
		printf("Excellent! Keep it up.\n");
		break;
	case 8:
		printf("Very Good! Well done.\n");
		break;
	case 7:
		printf("Good job! Keep improving.\n");
		break;
	case 6:
		printf("Fair. Need more effort.\n");
		break;
	default:
		printf("Poor performance. Study harder!\n");
	}
	return 0;
}
