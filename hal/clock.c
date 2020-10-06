/*******************************************************************************
  Clock Configuration Routine source File

  File Name:
    clock.c

  Summary:
    This file includes subroutine for initializing Oscillator and Reference 
    Clock Output

  Description:
    Definitions in the file are for dsPIC33EP256MC506 External OP-AMP PIM
    plugged onto Motor Control Development board from Microchip
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2019 released Microchip Technology Inc.  All rights reserved.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "clock.h"

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
// *****************************************************************************
/* Function:
    void InitOscillator(void)

  Summary:
    Routine to configure controller oscillator

  Description:
    Function configure oscillator PLL to generate desired processor clock

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitOscillator(void)
{
   /* Below equation provides relation between input frequency FIN(in MHz)and
     * Oscillator output frequency FOSC(in MHz)
     * Device Operating Frequency FCY(in MHz)is FOSC /2
     *
     *                    M                 (PLLDIV + 2)
     * FOSC  =  FIN * ---------- = -----------------------------------
     *                 (N1 * N2)    ((PLLPRE + 2)*(2 x (PLLPOST + 1)))
     *
     * where,
     * N1 = PLLPRE + 2
     * N2 = 2 x (PLLPOST + 1)
     * M = PLLDIV + 2
     */

#if (OSC_MODE == OSC_XTAL)

    /* If FIN (input frequency) = 8 MHz
     * FOSC (Oscillator output frequency)and FCY (Device Operating Frequency)is:
     *
     *                 M                   (68 + 2)            8 * 70
     * FOSC = FIN * --------- = 8 * ----------------------- = ------- = 140 MHz
     *              (N1 * N2)       ((0 + 2)*(2 x (0 + 1)))    2 * 2
     *
     * FCY = 140 MHz / 2 =  70 MHz
     *
     * where,
     * N1 = PLLPRE + 2 = 0 + 2 = 0
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2
     * M = PLLDIV + 2 = 68 + 2 = 70
     */

    /* PLLFBD: PLL FEEDBACK DIVISOR REGISTER(denoted as M,PLL multiplier)
     * M = (PLLFBDbits.PLLDIV+2)= (68+2) = 70                                 */
    PLLFBD = 68;

    /* PLL VCO Output Divider Select bits(denoted as N2, PLL post-scaler
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2                                   */
    CLKDIVbits.PLLPOST = 0;

    /* PLL Phase Detector I/P Divider Select bits(denoted as N1,PLL pre-scaler)
     * N1 = PLLPRE + 2 = 0 + 2 = 0                                            */
    CLKDIVbits.PLLPRE = 0;

    /* Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
     *  NOSC = 0b011 = Primary Oscillator with PLL (XTPLL, HSPLL, ECPLL)      */
    __builtin_write_OSCCONH(0x03);

    /* Request oscillator switch to selection specified by the NOSC<2:0>bits  */
    __builtin_write_OSCCONL(0x01);

    /* Wait for Clock switch to occur */
    while (OSCCONbits.COSC != 0b011);

    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);

#elif (OSC_MODE == OSC_FRC)

    /* If FIN (input frequency) = 7.37 MHz Internal RC Oscillator
     * FOSC (Oscillator output frequency)and FCY (Device Operating Frequency)is:
     *
     *         FIN * M         7.37 * (74 + 2)         7.37 * 76
     * FOSC = --------- = -----------------------  = ------------ = 140.03 MHz
     *        (N1 * N2)     ((0 + 2)*(2 x (0 + 1)))    (2 * 2)
     *
     * FCY = 140.03 MHz / 2 =  70.015 MHz
     *
     * where,
     * N1 = PLLPRE + 2 = 0 + 2 = 0
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2
     * M = PLLDIV + 2 = 74 + 2 = 76
     */

    /* PLLFBD: PLL FEEDBACK DIVISOR REGISTER(denoted as M,PLL multiplier)
     * M = (PLLFBDbits.PLLDIV+2)= (74+2) = 76                                 */
    PLLFBD = 74;

    /* PLL VCO Output Divider Select bits(denoted as N2, PLL post-scaler
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2                                   */
    CLKDIVbits.PLLPOST = 0;

    /* PLL Phase Detector I/P Divider Select bits(denoted as N1,PLL pre-scaler)
     * N1 = PLLPRE + 2 = 0 + 2 = 0                                            */
    CLKDIVbits.PLLPRE = 0;

    /* Initiate Clock Switch to FRC Oscillator with PLL (NOSC=0b001)
     *  NOSC = 0b001 = Fast RC Oscillator with PLL (FRCPLL)                   */
    __builtin_write_OSCCONH(0x01);

    /* Request oscillator switch to selection specified by the NOSC<2:0>bits  */
    __builtin_write_OSCCONL(0x01);

    /* Wait for Clock switch to occur */
    while (OSCCONbits.COSC != 0b001);

    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);

#endif
}

