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


/* Using typedef with struct can simplify the usage of structure types by allowing you
 * to omit the struct keyword when declaring variables of that type.
 * */

/* Define a structure named Person */
typedef struct{
	char name[50];
	int age;
	float height;
	char phone[15];
	char email[50];
	float weight;
}Person_t;
int main() {

	/* Declare and initialize a structure variable */
	    Person_t person1 = {"Ahmad", 30, 5.7, "123-456-7890","Ahmad@example.com", 65.0 };
//      Person_t person1 =  {.name = "adel", .age = 30, .height = 5.7, .phone = "123-456-7890", .email = "alice@example.com", .weight = 65.0};
	//    Person_t person3 =  {"mark", 30, 5.7, "123-456-7890", "alice@example.com", 65.0};

	/*Access and print structure members*/
	printf("Name: %s\n", person1.name);
	printf("Age: %d\n", person1.age);
	printf("Height: %.2f\n", person1.height);
	printf("Phone: %s\n", person1.phone);
	printf("Email: %s\n", person1.email);
	printf("Weight: %.2f\n", person1.weight);

	return 0;
}

