/*******************************************************************************
 ADC Configuration Routine Header File
 
  File Name:
    adc.h

  Summary:
    This header file lists ADC Configuration related functions and definitions.

  Description:
    Definitions in the file are for dsPIC33EP256MC506 External OP-AMP PIM
    plugged onto Motor Control Development board from Microchip
********************************************************************************/
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
#ifndef _ADC_H
#define _ADC_H

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
#include "pim_select.h"       
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// ADC MODULE Related Definitions
// Analog Channel No of Potentiometer #1 - used as Speed Reference
// POT1 : PIM #32 (AN13)
#define ANx_Vbus               10
// Analog Channel No of Inverter A DC bus voltage VDC_A
// VBUS_A : PIM #35 (AN10)
#define ANx_VBUS_A              10

#ifdef INTERNAL_OPAMP_PIM

#define ADCBUF_SPEED_REF_A      ADC1BUF0
#define ADCBUF_INV_A_IPHASE1    ADC1BUF2
#define ADCBUF_INV_A_IPHASE2    ADC1BUF3

#else

#define ADCBUF_SPEED_REF_A      ADC1BUF0
#define ADCBUF_INV_A_IPHASE1    ADC1BUF2
#define ADCBUF_INV_A_IPHASE2    ADC1BUF1
#define ADCBUF_INV_A_IBUS       ADC1BUF3

#endif

// ADC MODULE Related Definitions
// Specify Minimum ADC Clock Period (TAD)in uSec
#define ADC_MIN_TAD_MICROSEC     0.075
// Specify Max Time to stabilize Analog Stage from ADC OFF to ADC ON in uSecs
// The parameter, tDPU, is the time required for the ADC module to stabilize at
// the appropriate level when the module is turned on (AD1CON1<ADON> = 1).
// During this time, the ADC result is indeterminate.
#define ADC_TON_DELAY_MICROSEC  300

// Definitions for Channels 1, 2, 3 Positive Input Select bits
/* 001 = CH1 positive input is OA1(AN3),
         CH2 positive input is OA2(AN0),
         CH3 positive input is OA3(AN6) */
#define CH123_IS_OA1_OA2_OA3    0x01
/* 000 = CH1 positive input is AN0,
         CH2 positive input is AN1,
         CH3 positive input is AN2 */
#define CH123_IS_AN0_AN1_AN2    0x00

// Setting Channel No connected to ADC1 Sample/Hold Channel #0(ADC1-CH0)
// POT1 is connected for sample/conversion by ADC1 CH0
#define ADC1_ANx_CH0            ANx_Vbus
// Setting Channels to be connected to ADC1 Sample/Hold Channels 1,2,3
// for simultaneous sampling  : AN0(IB),AN1(IA),AN2(IBUS)
#define ADC1_ANx_CH123          CH123_IS_AN0_AN1_AN2
// Calculating  ADC conversion clock count ADCS from Min ADC Clock Period (TAD)
// TAD = Tp*(ADCS<7:0> + 1)= (1/Fp)*(ADCS<7:0> + 1)
// ADCS<7:0> = (MIN_TAD * Fp ) - 1 ~ (MIN_TAD * Fp )
// Subtraction by 1 is ignored as Min TAD cycle has to be met
#define ADC_MIN_ADCS_COUNTS     (uint8_t)((ADC_MIN_TAD_MICROSEC * FCY_MHZ))

#define EnableADC1Interrupt()   IEC0bits.AD1IE = 1
#define DisableADC1Interrupt()  IEC0bits.AD1IE = 0
#define ClearADC1IF()           IFS0bits.AD1IF = 0

/* This defines number of current offset samples for averaging 
 * If the 2^n samples are considered specify n(in this case 2^7(= 128)=> 7*/
#define  CURRENT_OFFSET_SAMPLE_SCALER         7 

// OPAMP/CMP MODULE Related Definitions
// Comparator Voltage Reference
#define CVREF_1    				0xF
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ADC initial offset storage data type

  Description:
    This structure will host parameters related to ADC module offset for all
    four sample and hold channels.
*/
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitADCModule1();
void InitAmplifiersComparator(void);

#ifdef __cplusplus
}
#endif

#endif      // end of ADC_H

