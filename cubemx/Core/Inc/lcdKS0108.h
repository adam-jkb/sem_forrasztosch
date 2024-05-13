/*! *******************************************************************************************************
* Copyright (c) 2023 K. Hugyik
*
* All rights reserved
*
* \file lcdKS0108.h
*
* \brief Functions that handle the display on the user interface panel
*
* \author K. Hugyik
*
**********************************************************************************************************/

#ifndef DISPLAY_H
#define DISPLAY_H

//--------------------------------------------------------------------------------------------------------/
// Include files
//--------------------------------------------------------------------------------------------------------/
#include <stdbool.h>

//--------------------------------------------------------------------------------------------------------/
// Definitions
//--------------------------------------------------------------------------------------------------------/


//--------------------------------------------------------------------------------------------------------/
// Types
//--------------------------------------------------------------------------------------------------------/


//--------------------------------------------------------------------------------------------------------/
// Global variables
//--------------------------------------------------------------------------------------------------------/


//--------------------------------------------------------------------------------------------------------/
// Interface functions
//--------------------------------------------------------------------------------------------------------/
void DISPLAY_Init(void);
void DISPLAY_Cycle(void);

void DISPLAY_Pixel(uint8_t u8PosX, uint8_t u8PosY, bool bIsOn);

void DISPLAY_DrawLine(uint8_t u8X0, uint8_t u8Y0, uint8_t u8X1, uint8_t u8Y1, bool bIsOn);
void DISPLAY_PrintChar(uint8_t u8Char, uint8_t u8X, uint8_t u8Y, bool bIsOn);
void DISPLAY_PrintString(uint8_t* pu8String, uint8_t u8X, uint8_t u8Y, bool bIsOn);

#endif  // DISPLAY_H

//-----------------------------------------------< EOF >--------------------------------------------------/