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

/* Structures in C are a powerful way to group different variables of
 * various data types under a single name. This makes it easier to
 *  manage related data as a single unit. */

/* Define a structure named Person */
struct Person {
	char name[50];
	int age;
	float height;
	char phone[15];
	char email[50];
	float weight;
};
int main() {

	/* Declare and initialize a structure variable */
	struct Person person1 = {"Ahmad", 30, 5.7, "123-456-7890","Ahmad@example.com", 65.0 };
    //  struct Person person2 =  {.name = "adel", .age = 30, .height = 5.7, .phone = "123-456-7890", .email = "alice@example.com", .weight = 65.0};
	//    struct Person person3 =  {"mark", 30, 5.7, "123-456-7890", "alice@example.com", 65.0};

	/*Access and print structure members*/
	printf("Name: %s\n", person1.name);
	printf("Age: %d\n", person1.age);
	printf("Height: %.2f\n", person1.height);
	printf("Phone: %s\n", person1.phone);
	printf("Email: %s\n", person1.email);
	printf("Weight: %.2f\n", person1.weight);

	return 0;
}

