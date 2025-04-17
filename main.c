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

	/* Description: Pointers are variables that store the memory address of another variable.
	 * Pointers are critical in embedded systems for interacting with hardware, managing memory, and optimizing performance.
		 * (*) Operator:
		 * Pointer Declaration: Used to declare a pointer variable.
		 * Dereference Operator: Used to access the value stored at the address pointed to by the pointer.
		 *
		 * (&) Operator:
		 * The ampersand & operator is used to get the address of a variable.
		 * It's also known as the "address-of" operator.
		 * */

	/*Pointer Declaration
	 * type *pointerName;
	 *  Initialize	pointer to 0, NULL, or	an	address. • 0 or NULL – points to nothing (NULL preferred)
	 * */
	    int a = 10;
	    int *p = &a;// a p set to address of a

	    printf("Value of a: %d\n", a);
	    printf("Address of a: %p\n", &a);
	    printf("Pointer p holds address: %p\n", p);
	    printf("Value pointed by p: %d\n", *p);

	    /*Changing value using pointer*/
	    *p = 20;
	    printf("New value of a: %d\n", a);
	    *p = 2 * *p - a;
	    printf("New value of a after operator: %d\n\n", a);

/*******************************************************************************/
    int arr[5] = {10, 20, 30, 40, 50};
    int *ptr = arr;  // Pointer to the first element of the array

    printf("Original array elements:\n");
    for (int i = 0; i < 5; i++) {
        printf("arr[%d] = %d\n", i, arr[i]);
    }

    // Modify array elements using the pointer
    for (int i = 0; i < 5; i++) {
        *(ptr + i) = *(ptr + i) * 2;  // Double each element

        //arr[i] = arr[i] * 2;
    }

    printf("\nModified array elements:\n");
    for (int i = 0; i < 5; i++) {
        printf("arr[%d] = %d\n", i, arr[i]);
    }

	return 0;
}
