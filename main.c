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

	int8_t a = 5;  // 0101 in binary
	int8_t b = 3;  // 0011 in binary

	    int resultAND = a & b; // 0001 in binary
	    /* 1- Bitwise AND (&): Used to check if specific bits are set (1) or cleared (0) */
	    printf("AND Result: %d\n\n", resultAND); // Output: 1

	    int resultOR = a | b; // 0111 in binary
	    /* 2- Bitwise OR (|): Used to set specific bits */
	    printf("OR Result: %d\n\n", resultOR); // Output: 7

	    int resultNOT = ~a; // 1010 in binary (Two's complement: -6)
	    /* 3- Bitwise NOT (~): Used to invert all bits. */
	    printf("NOT Result: %d\n\n", resultNOT); // Output: -6

	    int resultXOR = a ^ b; // 0110 in binary
	    /* 4- Bitwise XOR (^): Used to toggle specific bits */
	    printf("XOR Result: %d\n\n", resultXOR); // Output: 6

	    int resultShiftLeft = a << 2; // 010100 in binary
	    /* 5- Bitwise Left Shift (<<): Used to shift bits to higher positions (left) */
	    printf("Left Shift Result: %d\n\n", resultShiftLeft); // Output: 20

	    int resultShiftRight = a >> 2; // 101 in binary
	    /* 6- Bitwise Right  Shift (<<): Used to shift bits to lower positions (right) */
	    printf("Right Shift Result: %d\n\n", resultShiftRight); // Output: 5

/********************************************************************************/
	    /*Bit Manipulations for Control Operations
	     * Setting or clearing specific bits,
	     *  */
	    uint8_t controlRegister = 0b00001111; // Initial state (15)
	    // Set the 6th bit
	    controlRegister |= (1 << 5);
	    printf("controlRegister SET: %d\n\n", controlRegister);
	    // Clear the 3rd bit
	    controlRegister &= ~(1 << 2);
	    printf("controlRegister ERSET: %d\n\n", controlRegister);
	    /* Check if flag at position 4 is set*/
	    bool isSet = controlRegister & (1 << 2);
	    printf("Value of isSet: %d\n\n", isSet);

/********************************************************************************/
       /* Extracting Bytes from uint32_t to uint8_t*/
	    /*Consider you have a uint32_t value, and you need to store each byte in an array of uint8_t.*/
	    int64_t bigValue = 0x12345678;
	    uint8_t byteArray[4];
	    for (int i = 0; i < 4; i++) {
	    byteArray[i] = (bigValue >> (i * 8)) & 0xFF;
	    printf("byteArray[%d] = ox%x\n",i,byteArray[i]);
	    }

       /* Combining Multiple uint8_t Values into a uint32_t*/
	    /*Imagine a situation where you need to combine four uint8_t sensor readings into a single uint32_t value for
         *efficient transmission.
         **/
	    uint8_t sensor1 = 0x12; // Example sensor values
	    uint8_t sensor2 = 0x34;
	    uint8_t sensor3 = 0x56;
	    uint8_t sensor4 = 0x78;
	    int32_t combinedSensors = ((uint32_t)(sensor1) << 24) | ((uint32_t)(sensor2) << 16) | ((uint32_t)(sensor3) << 8) | (uint32_t)(sensor4);
	    printf("\ncombinedSensors = ox%x\n",combinedSensors);
	return 0;
}

