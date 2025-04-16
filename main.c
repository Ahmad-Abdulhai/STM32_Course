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
/*
 *Variable Declaration Format:
 * type-qualifier(s) type-modifier data-type variable-name = initial-value;
 */

/*1- Example Declaration and Assignment of data type:*/

/*A declaration tells the compiler that a variable exists, and what type it is.
* It does not necessarily assign a value.
 */
int foo; //Declares a variable 'foo' of type int
int a = 10;    // Declaration + Initialization
float floatVar = 12.564643f ;
double doubleVar = 12.232143;

/*************************************************************************************/
/*2- Modifier of data type*/
unsigned char uCharVar = 255; // Modifier data range to [ 0 --> 255]
short int sIntVar = -32000 ; // Modifier data range from 4 bytes to 2 bytes
unsigned short int uSintVar = 65535; // Modifier data range to [ 0 --> 65535]
long LongVar = 0x7FFFFFFF;
long long doubleLongVar = 0x7FFFFFFFFFFFFFFF; //// Modifier data range to 8 bytes

/*************************************************************************************/
/*3- Qualifier of data type*/
/*
 * Const: Const variables are typically stored in ROM or (flash) memory-Read only- if they are global or static.
 * The compiler protects ‘const’ definitions of INADVERTENT WRITING
 */
/*
 * volatile: Find it yourself :)
 * */
const unsigned char maxSize =100 ;
const unsigned int ADC_MAX_VALUE = 4095;
const float PI = 3.141593f ;
int main() {

	/*1-Example Declaration and Assignment of data type*/
    printf("Value of variable 'foo' before  Initialization :%d\n",foo);
    printf("Value of variable 'a' after  Initialization :%d\n",a);
    /* Assignment a value to 'foo' variable*/
    foo = -15 ;
    printf("Value of variable 'foo' after  Assignment :%d\n",foo);
    printf("Value of variable 'floatVar':%f | Size 'float':%d bytes \n",floatVar, sizeof(floatVar));
    printf("Value of variable 'doubleVar':%f | Size 'double':%d bytes \n\n",doubleVar, sizeof(doubleVar));

/*************************************************************************************/
    /*2- Modifier of data type*/
    printf("Value of variable 'uCharVar':%d | Size 'unsigned char':%d bytes \n",uCharVar,sizeof(uCharVar));
    printf("Value of variable 'sIntVar':%d | Size 'short int':%d bytes \n",sIntVar,sizeof(sIntVar));
    printf("Value of variable 'uSintVar':%d | Size 'unsigned short int':%d bytes \n",uSintVar,sizeof(uSintVar));
    printf("Value of variable 'LongVar':%ld bytes | Size 'Long':%d bytes \n",LongVar ,sizeof(LongVar));

/*************************************************************************************/
    /*3- -Qualifier of data type*/
    printf("Value of const variable 'PI':%f\n",PI);
    return 0;
}
