/*******************************************************************************
   Header File for High-Resolution PWM with Fine Edge Placement Configuration.

  File Name:
    pwm.h

  Summary:
    This header file lists routines to configure High-Resolution PWM with Fine 
    Edge Placement.

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
#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "clock.h"
#include <libpic30.h>
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// MC PWM MODULE Related Definitions
#define INVERTERA_PWM_PDC1      PDC1
#define INVERTERA_PWM_PDC2      PDC2
#define INVERTERA_PWM_PDC3      PDC3

/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         20000
/* Specify dead time in micro seconds */
#define DEADTIME_MICROSEC       1.0
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC            0.00005
/* Specify PWM Period in micro seconds */
#define LOOPTIME_MICROSEC       50
// Calculating deadtime in units of Tcy(Centre aligned Mode)
#define PWM_DT_ERRATA

#define DDEADTIME               (uint16_t)(2*DEADTIME_MICROSEC*FCY_MHZ)
// Basic loop period in units of Tcy
#define LOOPTIME_TCY            (uint16_t)(FCY_HZ/PWMFREQUENCY_HZ)

#ifdef PWM_DT_ERRATA
// Should be >= DDEADTIME/2 for PWM Errata workaround
    #define MIN_DUTY            (uint16_t)(DDEADTIME/2 + 1)
#else
    #define MIN_DUTY            0x0000
#endif

// Specify bootstrap charging time in no of us
#define BOOTSTRAP_CHARGING_TIME 30000

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitPWMGenerators(void);        
// *****************************************************************************
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif      // end of PWM_H


