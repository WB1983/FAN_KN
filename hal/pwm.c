/*******************************************************************************
  Source File for High-Resolution PWM with Fine Edge Placement Configuration.

  File Name:
    pwm.c

  Summary:
    This file includes subroutine for initializing  High-Resolution PWM with 
    Fine Edge Placement.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "pwm.h"
#include "userparms.h"
#include "delay.h"
#include "pim_select.h"

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

void InitPWM123Generators(void);
void InitPWMGenerators(void);
void ChargeBootstarpCapacitors(void);


// *****************************************************************************
/* Function:
    InitPWMGenerators()

  Summary:
    Routine to initialize PWM Module for Inverters

  Description:
    Function initializes  and enable the PWM Module after configuration

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPWMGenerators(void)
{
    // PTCON: PWMx TIME BASE CONTROL REGISTER
    // PTEN(15): PWMx Module Enable bit
    // Unimplemented(14): Read as ?0?
    // PTSIDL(13): PWMx Time Base Stop in Idle Mode bit
    // SESTAT(12): Special Event Interrupt Status bit
    // SEIEN(11): Special Event Interrupt Enable bit
    // EIPU(1): Enable Immediate Period Updates bit(1)
    // SYNCPOL(9): Synchronize Input and Output Polarity bit(1)
    // SYNCOEN(8): Primary Time Base Sync Enable bit(1)
    // SYNCEN(7): External Time Base Synchronization Enable bit
    // SYNCSRC<(6-4): Synchronous Source Selection bits(1)
    // SEVTPS(3-0): PWMx Special Event Trigger Output Post-scaler Select bits
    PTCON = 0;               // 0b0000 0000 0000 0000(PWM Module is disabled)
    PTCONbits.EIPU = 0;      // 0 = Active Period register updates occur
                             // on PWMx cycle boundaries

    // PTCON2: PWMx PRIMARY MASTER CLOCK DIVIDER SELECT REGISTER
    // Unimplemented(15-3): Read as ?0?
    // PCLKDIV(2-0): PWMx Input Clock Pre-scaler (Divider) Select bits
    PTCON2 = 0;              // 0b0000 0000 0000 0000(PWM Module is disabled)
    PTCON2bits.PCLKDIV = 0;  // 0b000 = Divide-by-1, max PWM timing resolution

    InitPWM123Generators();

    ChargeBootstarpCapacitors();

    PTCONbits.PTEN = 1;      // Enable PWM module after initializing generators


    return;
}

// *****************************************************************************
/* Function:
    InitPWM123Generators()

  Summary:
    Routine to initialize PWM generators 1,2,3 

  Description:
    Function initializes PWM module for 3-phase inverter control in Complimentary
    mode ;initializes period,dead time;Configures PWM fault control logic

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPWM123Generators(void)
{
    // PWMCONx: PWMx CONTROL REGISTER
    // FLTSTAT(15): Fault Interrupt Status bit
    // CLSTAT(14): Current-Limit Interrupt Status bit
    // TRGSTAT(13): Trigger Interrupt Status bit
    // FLTIEN(12): Fault Interrupt Enable bit
    // CLIEN(11): Current-Limit Interrupt Enable bit
    // TRGIEN(10): Trigger Interrupt Enable bit
    // ITB(9): Independent Time Base Mode bit
    // MDCS(8): Master Duty Cycle Register Select bit
    // DTC(7-6): Dead-Time Control bits
    // DTCP(5): Dead-Time Compensation Polarity bit
    // Unimplemented(4-3): Read as ?0?
    // CAM(2): Center-Aligned Mode Enable bit
    // XPRES(1): External PWMx Reset Control bit
    // IUE(0): Immediate Update Enable bit
    PWMCON1 = 0;                // 0b0000 0000 0000 0000
    PWMCON2 = 0;                // 0b0000 0000 0000 0000
    PWMCON3 = 0;                // 0b0000 0000 0000 0000

    PWMCON1bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled
    PWMCON2bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled
    PWMCON3bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled

    PWMCON1bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled
    PWMCON2bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled
    PWMCON3bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled

    PWMCON1bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled
    PWMCON2bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled
    PWMCON3bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled

    PWMCON1bits.ITB = 1;        // 1 = PHASE1 register provides time base period
    PWMCON2bits.ITB = 1;        // 1 = PHASE2 register provides time base period
    PWMCON3bits.ITB = 1;        // 1 = PHASE3 register provides time base period

    PWMCON1bits.MDCS = 0;       // 0 = PDC1 register provides duty cycle
    PWMCON2bits.MDCS = 0;       // 0 = PDC2 register provides duty cycle
    PWMCON3bits.MDCS = 0;       // 0 = PDC3 register provides duty cycle

    PWMCON1bits.DTC = 0;        // 00 = Positive dead time is actively applied
    PWMCON2bits.DTC = 0;        // 00 = Positive dead time is actively applied
    PWMCON3bits.DTC = 0;        // 00 = Positive dead time is actively applied

    PWMCON1bits.CAM = 1;        // 1 = Center-Aligned mode is enabled
    PWMCON2bits.CAM = 1;        // 1 = Center-Aligned mode is enabled
    PWMCON3bits.CAM = 1;        // 1 = Center-Aligned mode is enabled

    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC1/DTR1/ALTDTR1/PHASE1 registers are
                                // synchronized to the PWM1 period boundary
    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC2/DTR2/ALTDTR2/PHASE2 registers are
                                // synchronized to the PWM2 period boundary
    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC3/DTR3/ALTDTR3/PHASE3 registers are
                                // synchronized to the PWM3 period boundary

    // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER
    // PHASEx register provides time base period, if PWMCONxbits.ITB = 1
    PHASE1 = LOOPTIME_TCY;      // Setting PWM period for PWM1 generator
    PHASE2 = LOOPTIME_TCY;      // Setting PWM period for PWM2 generator
    PHASE3 = LOOPTIME_TCY;      // Setting PWM period for PWM3 generator
    
    // DTRx: PWMx DEAD-TIME REGISTER
    DTR1 = 0x0000;              // Not used in Center-Aligned mode
    DTR2 = 0x0000;              // Not used in Center-Aligned mode
    DTR3 = 0x0000;              // Not used in Center-Aligned mode

    // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER
    // Provides 14-Bit Dead-Time Value for PWMx generator if PWMCONxbits.CAM = 1
    ALTDTR1 = DDEADTIME;        // Setting dead-time for PWM1 generator
    ALTDTR2 = DDEADTIME;        // Setting dead-time for PWM2 generator
    ALTDTR3 = DDEADTIME;        // Setting dead-time for PWM3 generator

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialize the PWM duty cycle register
    PDC1 = MIN_DUTY;
    PDC2 = MIN_DUTY;
    PDC3 = MIN_DUTY;

    // IOCONx: PWMx I/O CONTROL REGISTER
    // PENH(15): PWMxH Output Pin Ownership bit
    // PENL(14): PWMxL Output Pin Ownership bit
    // POLH(13): PWMxH Output Pin Polarity bit
    // POLL(12): PWMxL Output Pin Polarity bit
    // PMOD(11:10): PWMx I/O Pin Mode bits
    // OVRENH(9): Override Enable for PWMxH Pin bit
    // OVRENL(8): Override Enable for PWMxL Pin bit
    // OVRDAT(7:6): Data for PWMxH, PWMxL Pins if Override is Enabled bits
    // FLTDAT(5:4): Data for PWMxH and PWMxL Pins if FLTMOD is Enabled bits.
    // CLDAT(1:0): Data for PWMxH and PWMxL Pins if CLMOD is Enabled bits
    // SWAP(1): SWAP PWMxH and PWMxL Pins bit
    // OSYNC(1): Output Override Synchronization bit
    IOCON1 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM1H,L pins)
    IOCON2 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM2H,L pins)
    IOCON3 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM3H,L pins)

    IOCON1bits.PENH = 1;    // 1 = PWM1H pin is active-high
    IOCON2bits.PENH = 1;    // 1 = PWM2H pin is active-high
    IOCON3bits.PENH = 1;    // 1 = PWM3H pin is active-high

    IOCON1bits.PENL = 1;    // 1 = PWM1L pin is active-high
    IOCON2bits.PENL = 1;    // 1 = PWM2L pin is active-high
    IOCON3bits.PENL = 1;    // 1 = PWM3L pin is active-high

    IOCON1bits.POLH = 0;    // 0 = PWM1H pin is active-high
    IOCON2bits.POLH = 0;    // 0 = PWM2H pin is active-high
    IOCON3bits.POLH = 0;    // 0 = PWM3H pin is active-high

    IOCON1bits.POLL = 0;    // 0 = PWM1L pin is active-high
    IOCON2bits.POLL = 0;    // 0 = PWM2L pin is active-high
    IOCON3bits.POLL = 0;    // 0 = PWM3L pin is active-high

    IOCON1bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM1H,L is driven LOW
    IOCON2bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM2H,L is driven LOW
    IOCON3bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM3H,L is driven LOW

    IOCON1bits.PMOD = 0;    // 0b00 = PWM1 pair is in the Complementary O/P mode
    IOCON1bits.PMOD = 0;    // 0b00 = PWM2 pair is in the Complementary O/P mode
    IOCON1bits.PMOD = 0;    // 0b00 = PWM3 pair is in the Complementary O/P mode

    // TRGCONx: PWMx TRIGGER CONTROL REGISTER
    // TRGDIV (15-12): Trigger Output Divider bits
    // Unimplemented(11-6): Read as ?0?
    // TRGSTRT(5:0): Trigger Post-scaler Start Enable Select bits
    TRGCON1 = 0x0000;       // 0b0000 UUUU UU00 0000
    TRGCON2 = 0x0000;       // 0b0000 = Trigger output for every trigger event
    TRGCON3 = 0x0000;       // 0b0000 = Trigger output for every trigger event

    TRGCON1bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event
    TRGCON2bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event
    TRGCON3bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event

    TRGCON1bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled
    TRGCON2bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled
    TRGCON3bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled

    // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER
    // TRGCMP(15:0): Trigger Control Value bits
    // When the primary PWMx functions in local time base, 
    // this register contains the compare values that can trigger the ADC module.
    TRIG1 = PHASE1 - 1;         // Set to trigger ADC at the PWM edge
    TRIG2 = 0;                  // Set at center of the PWM (not used)
    TRIG3 = 0;                  // Set at center of the PWM (not used)

    // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER
    // Unimplemented(15): Read as '0'
    // CLSRC(14:10): Current-Limit Control Signal Source Select 
    // CLPOL(9): Current-Limit Polarity for PWM Generator bit
    // CLMOD (8): Current-Limit Mode Enable for PWM Generator bit
    // FLTSRC(7:3): Fault Control Signal Source Select for PWM Generator bits
    // FLTPOL (2): Fault Polarity for PWM Generator bit
    // FLTMOD (1:0): Fault Mode for PWM Generator bits 
    FCLCON1 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled) 
    FCLCON2 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled)
    FCLCON3 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled)

#ifdef INTERNAL_OPAMP_PIM
    
    FCLCON1bits.FLTSRC = 0x0B;  // 0b01011 = Fault SRC is Comparator4
    FCLCON2bits.FLTSRC = 0x0B;  // 0b01011 = Fault SRC is Comparator4
    FCLCON3bits.FLTSRC = 0x0B;  // 0b01011 = Fault SRC is Comparator4

#else

    FCLCON1bits.FLTSRC = 0x1F;  // 0b11111 = Fault SRC is FLT32
    FCLCON2bits.FLTSRC = 0x1F;  // 0b11111 = Fault SRC is FLT32
    FCLCON3bits.FLTSRC = 0x1F;  // 0b11111 = Fault SRC is FLT32

#endif

    FCLCON1bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low
    FCLCON2bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low
    FCLCON3bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low

    FCLCON1bits.FLTMOD = 1;     // 0b01 = forces PWM1H,L to FLTDAT values(cycle)
    FCLCON2bits.FLTMOD = 1;     // 0b01 = forces PWM2H,L to FLTDAT values(cycle)
    FCLCON3bits.FLTMOD = 1;     // 0b01 = forces PWM3H,L to FLTDAT values(cycle)

    // Configuring PWMx Generators Interrupts and Priority
    IPC23bits.PWM1IP = 4;       // PWM1 Interrupt Priority is 4
    IPC23bits.PWM2IP = 4;       // PWM2 Interrupt Priority is 4
    IPC24bits.PWM3IP = 4;       // PWM3 Interrupt Priority is 4

    IFS5bits.PWM1IF = 0;        // Clear PWM1 Interrupt flag
    IFS5bits.PWM2IF = 0;        // Clear PWM1 Interrupt flag
    IFS6bits.PWM3IF = 0;        // Clear PWM1 Interrupt flag

    IEC5bits.PWM1IE = 0;        // Disable PWM1 Interrupt
    IEC5bits.PWM2IE = 0;        // Disable PWM2 Interrupt
    IEC6bits.PWM3IE = 0;        // Disable PWM3 Interrupt

    return;
}

// *****************************************************************************
/* Function:
    ChargeBootstarpCapacitors()

  Summary:
    Routine to initialize PWM generators 1,2,3 to charge bootstrap capacitors

  Description:
    Function to charge bootstrap capacitors

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ChargeBootstarpCapacitors(void)
{
    uint16_t i = BOOTSTRAP_CHARGING_TIME;

    // Enable PWMs only on PWMxL ,to charge bootstrap capacitors initially
    // Hence PWMxH is over-ridden to "LOW"
    IOCON1bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled
    IOCON2bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled
    IOCON3bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled

    IOCON1bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H
    IOCON2bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H
    IOCON3bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H

    IOCON1bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM1L pin
    IOCON2bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM2L pin
    IOCON3bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM3L pin

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialize the PWM duty cycle for charging
    PDC1 = PHASE1 - (MIN_DUTY + 3);
    PDC2 = PHASE1 - (MIN_DUTY + 3);
    PDC3 = PHASE1 - (MIN_DUTY + 3);

    PTCONbits.PTEN = 1;      // Enable PWM module for charging bootstrap CAPs

    while(i)
    {
        __delay_us(1);
        i--;
    }

    PTCONbits.PTEN = 0;      // Disable PWM module after charging

    // Reset modified PWM configurations after bootstrap charging
    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Re-initialise the PWM duty cycle register
    PDC1 = MIN_DUTY;
    PDC2 = MIN_DUTY;
    PDC3 = MIN_DUTY;

    IOCON1bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM1H pin
    IOCON2bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM2H pin
    IOCON3bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM3H pin
}