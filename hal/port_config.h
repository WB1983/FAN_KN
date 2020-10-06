/*******************************************************************************
  Hardware specific routine definition and interfaces Header File.

  File Name:
    port_config.h

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins.

  Description:
    Definitions in the file are for dsPIC33EP256MC506 External OP-AMP PIM
    plugged onto Motor Control Development board from Microchip.
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2020 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

#include <xc.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
       
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// Digital I/O definitions
// Push button Switches
#ifdef MCLV2
// SW1 : PIM #83 (RG7)
#define S2                  !PORTGbits.RG7
// SW2 : PIM #84 (RG6)
#define S3                  !PORTGbits.RG6

// SW1 : PIM #69 - Used as START/STOP button of Motor A
#define BUTTON_START_STOP          S2
// SW2 : PIM #84 - Used as Speed HALF/DOUBLE button of Motor A
#define BUTTON_SPEED_HALF_DOUBLE   S3
#endif

#ifdef MCHV2_MCHV3
// PUSH BUTTON: PIM #68
#define PUSHBUTTON PORTBbits.RB8   

// PUSH BUTTON : PIM #68 - Used as START/STOP button of Motor
#define BUTTON_START_STOP        PUSHBUTTON 
#endif
// Debug LEDs
// LED2 (D2) : PIM #01(RD6)
#define LED2                    LATDbits.LATD6
// LED1 (D17 on MCLV-2 / D19 on MCHV-2) : PIM #60(RD5)
#define LED1                    LATDbits.LATD5

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void SetupGPIOPorts(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
#ifdef __cplusplus
}
#endif

#endif      // end of PORTCONFIG_H