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


    /* for Loop:
     * Description: The for loop is used when the number of iterations is known beforehand.
     * It's typically used for counting or iterating over a sequence.
     */
    printf("for Loop:\n");
    for (int i = 1; i <= 5; i++) {
        printf("Iteration %d\n", i);
    }
/*******************************************************************************/

    /* while Loop:
     * Description: The while loop is used when the number of iterations
     * is not known beforehand and depends on a condition.
     * The loop checks the condition before executing the block of code.
     */
    int j = 5;
    printf("\nwhile Loop:\n");
    while (j > 0) {
        printf("Iteration %d\n", j);
        j--;
    }
/*******************************************************************************/

    /* do-while Loop:
     * Description: The do-while loop is similar to the while loop,
     * but it guarantees that the block of code is executed at least once.
     * The condition is checked after the code block is executed.
     */
    int k = 5;
    printf("\ndo-while Loop:\n");
    do {
        printf("Iteration %d\n", k);
        k--;
    } while (k > 0);


	return 0;
}
