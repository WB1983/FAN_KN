/*******************************************************************************
  ADC Configuration Routine source File

  File Name:
    adc.c

  Summary:
    This file includes subroutine for initializing ADC Cores of Controller.

  Description:
    Definitions in the file are for dsPIC33EP256MC506 External OP-AMP PIM
    plugged onto Motor Control Development board from Microchip
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
#include "adc.h"
#include "delay.h"

void InitADCModule1();
void InitAmplifiersComparator(void);
// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    InitADCModule1()

  Summary:
    Routine to initialize ADC 

  Description:
    Function initializes the ADC1 for simultaneously sampling four AD channels
    Measures offset on all channels .

  Precondition:
    None.

  Parameters:
    Pointer to ADC_OFFSET_T structure variable created to store ADC1 offset is
    passed to this function

  Returns:
    Structure will contain the averaged initial offset value of all 4 channels

  Remarks:
    None.
 */
void InitADCModule1()
{
    // AD1CON1: ADC1 CONTROL REGISTER 1
    // ADON(15): ADC Operating Mode bit
    // Unimplemented(14,11): Read as '0'
    // ADSIDL(13): ADC Stop in Idle Mode bit
    // ADDMABM(12): DMA Buffer Build Mode bit
    // AD12B(10): 10-Bit or 12-Bit ADC Operation Mode bit
    // FORM(9-8): Data Output Format bits
    // SSRC(7-5>: Sample Clock Source Select bits
    // SSRCG(4): Sample Trigger Source Group bit
    // SIMSAM(3): Simultaneous Sample Select bit (only if CHPS<1:0> = 01 or 1x)
    // In 12-Bit Mode (AD12B = 1), SIMSAM is Unimplemented and is Read as '0'
    // ASAM(2): ADC Sample Auto-Start bit
    // SAMP(1): ADC Sample Enable bit
    // DONE(0): ADC Conversion Status bit
    AD1CON1 = 0;            // 0b0U00 U000 0000 0000 (0 = ADC is OFF)
    AD1CON1bits.AD12B = 0;   // 0 = 10-bit, 4-channel ADC operation possible
    AD1CON1bits.FORM = 3;   // 11 = Signed fractional(DOUT = sddd dddd dd00 0000
                            // Range is shown below for 10bit operation:
                            // 0b0111 1111 1100 0000 = 0.99804
                            // 0b0000 0000 0000 0000 = 0
                            // 0b1000 0000 0000 0000 = -1
    AD1CON1bits.SSRCG = 1;  // if SSRCG =  1,
    AD1CON1bits.SSRC = 0;   // 000 = PWM Generator 1 primary trigger compare
                            // ends sampling and starts conversion
    AD1CON1bits.SIMSAM = 1; // Samples CH0-CH3 simultaneously(if CHPS = 1x).
    AD1CON1bits.ASAM = 1;   // Sampling begins immediately after
                            // last conversion completes.SAMP bit is auto set.
    AD1CON1bits.DONE = 0;   // ADC conversion has not started

    // ADxCON2: ADCx CONTROL REGISTER 2
    // VCFG(15-13): Converter Voltage Reference Configuration bits
    // Unimplemented(12-11): Read as ?0?
    // CSCNA(10): Input Scan Select bit
    // CHPS(9-8): Channel Select bits
    // In 12-Bit Mode,CHPS<1:0> Bits are Unimplemented and are Read as ?0?
    // BUFS(7): Buffer Fill Status bit (only valid when BUFM = 1)
    // SMPI(6-2): Increment Rate bits
    // BUFM(1): Buffer Fill Mode Select bit
    // ALTS(0): Alternate Input Sample Mode Select bit
    AD1CON2 = 0;
    AD1CON2bits.VCFG = 0;   // ADC Voltage Reference VREFH = AVDD; VREFL = AVSS
    AD1CON2bits.CSCNA = 0;  // 0 = Does not scan inputs
    AD1CON2bits.CHPS = 2;   // 1x = Converts CH0, CH1, CH2 and CH3
    AD1CON2bits.SMPI = 0;   // x0000 = Generates interrupt after completion
                            // of every sample/conversion operation
    AD1CON2bits.BUFM = 0;   // 0 = Starts filling buffer from the Start address
    AD1CON2bits.ALTS = 0;   // 0 = Uses channel I/P selects for Sample MUXA

    // ADxCON3: ADCx CONTROL REGISTER 3
    // ADRC(15): ADC Conversion Clock Source bit
    // Unimplemented(14-13): Read as ?0?
    // SAMC(12-8): Auto-Sample Time bits
    // ADCS(7-0): ADC Conversion Clock Select bits
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0;       // 0 = Clock derived from system clock
    // ADC Conversion Clock Select bits
    // Tp*(ADCS<7:0> + 1) = Tp*(6+1) = 7 Tp = 1 Tad
    // 7 * (1/ Fp) = 7 * (1 / Fcy) = 100nSec
    AD1CON3bits.ADCS = ADC_MIN_ADCS_COUNTS;

    // ADxCON4: ADCx CONTROL REGISTER 4
    // Unimplemented(15-9,7-3): Read as ?0?
    // ADDMAEN(8): ADC DMA Enable bit
    // DMABL(2-0): Selects Number of DMA Buffer Locations per Analog Input bits
    AD1CON4 = 0;                   // Not used in the application

    // ADxCHS123: ADCx INPUT CHANNEL 1, 2, 3 SELECT REGISTER
    // Unimplemented(15-11,7-3): Read as '0'
    // CH123SB(8): Channels 1, 2, 3 Positive Input Select for Sample B bits
    // CH123NB(10:9): Channels 1, 2, 3 Negative Input Select for Sample B bits
    // CH123SA(0): Channels 1, 2, 3 Positive Input Select for Sample A bits
    // CH123NA(2-1): Channels 1, 2, 3 Negative Input Select for Sample A bits
    AD1CHS123 = 0x0000;            // 0bUUUU U000 UUUU U000
    AD1CHS123bits.CH123SB = 0;     // Not used int the application
    AD1CHS123bits.CH123NB = 0;     // 0x = CH1, CH2, CH3 negative input is VREFL

    // Sets the simultaneously sampled channels
    // Refer adc.h for details
#ifdef INTERNAL_OPAMP_PIM
    // ADC setup for simultaneous sampling on 
    //      CH0=AN13, CH1=CMP1, CH2=CMP2, CH3=CMP3.
    AD1CHS123bits.CH123SA = CH123_IS_OA1_OA2_OA3;
#else    
    // ADC setup for simultaneous sampling on 
    //      CH0=AN13, CH1=AN0, CH2=AN1, CH3=AN2. 
    AD1CHS123bits.CH123SA = CH123_IS_AN0_AN1_AN2;
#endif    
    
    AD1CHS123bits.CH123NA = 0;     // 0x = CH1, CH2, CH3 negative input is VREFL
    
    // AD1CHS0: ADC1 INPUT CHANNEL 0 SELECT REGISTER
    // CH0NB(15): Channel 0 Negative Input Select for Sample MUXB bit
    // CH0SB(13-8): Channel 0 Positive Input Select for Sample MUXB bits(1,4,5)
    // CH0NA(7): Channel 0 Negative Input Select for Sample MUXA bit
    // CH0SA(5:0): Channel 0 Positive Input Select for Sample MUXA bits(1,4,5)
    // Unimplemented(14,6): Read as '0'
    AD1CHS0 = 0;
    AD1CHS0bits.CH0NB = 0;  // 0 = Channel 0 negative input is VREFL
    AD1CHS0bits.CH0SB = 0;  // 0b000000 (Not used in this application)
    AD1CHS0bits.CH0NA = 0;  // 0 = Channel 0 negative input is VREFL
    // CH0 is used for measuring voltage set by speed reference potentiometer
    AD1CHS0bits.CH0SA = ADC1_ANx_CH0;
    
    // AD1CSSH: ADC1 INPUT SCAN SELECT REGISTER HIGH
    // CSS(31-16): ADC Input Scan Selection bits
    AD1CSSH = 0;

    // AD1CSSL: ADC1 INPUT SCAN SELECT REGISTER LOW
    // CSS(15-0): ADC Input Scan Selection bits
    AD1CSSL = 0;

    // Configuring ADCx Generators Interrupts and Priority
    IPC3bits.AD1IP = 7;         // 0b111 ADC1 Interrupt Priority is 7
    IFS0bits.AD1IF = 0;         // 0 = Clear ADC1 Interrupt flag
    IEC0bits.AD1IE = 0;         // 0 = Disable ADC1 Interrupt

    AD1CON1bits.ADON = 1;

    // Time to Stabilize Analog Stage from ADC Off to ADC On
    __delay_us(ADC_TON_DELAY_MICROSEC);
}

/* Function:
    InitAmplifiersComparator()

  Summary:
    Routine to initialize Op-Amps and Comparator

  Description:
    Function initialize Op-Amps for motor current amplification;comparators for
    over current detection

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitAmplifiersComparator(void)
{
#ifdef INTERNAL_OPAMP_PIM
   // Initialize OPAMP/Comparator 
   // OpAmp 1,2,3 for Signal Conditioning Currents & Comparator 4 for Fault Generation

                                // C1IN1+ :IBUS+ ; C1IN1- : IBUS- 
    CM1CON = 0x8C00;			// OpAmp1- OA1OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C1IN1- pin
                                // C2IN1+ :IA+ ; C2IN1- : IBUS+ 
    CM2CON = 0x8C00;			// OpAmp2- OA2OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C2IN1- pin
                                // C3IN1+ :IB+ ; C3IN1- : IBUS+ 
    CM3CON = 0x8C00;			// OpAmp3- OA3OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C3IN1- pin
    
    CM4CON = 0x8011;            // comparator output non inverted
                                // VIN+ input connects to internal CVREFIN voltage
                                // VIN- input connects to CMP1 (source Ibus) 
 
    CVRCON = 0x008F;            // CVREF = (0.1031)*CVR + 0.825, CVR = 15
    CM4FLTR = 0x000F;           // max filtering
#else 
    // CVR1CON: COMPARATOR VOLTAGE REFERENCE CONTROL REGISTER 1
    // CVREN <15>: 1 = Comparator voltage reference circuit powered on
    // CVROE <14>: 0 = CVREF voltage level is disconnected from CVREF10 pin
    // CVRSS <11>: 0 = Comparator reference source CVRSRC = AVDD-AVSS
    // VREFSEL <10>: 0 = CVREFIN is generated by the resistor network
    // Unimplemented <13,12,9,8,7>: Read as ?0?
    // CVR<6:0>: Comparator Voltage Reference Value Selection bits
    // CVREF = (CVR<6:0> / 128)* CVRSRC
    CVRCON = 0x0000;
 //   CVR1CONbits.CVR = CVREF_1;
 //   CVR1CONbits.CVREN = 1;

    // CVR2CON: COMPARATOR VOLTAGE REFERENCE CONTROL REGISTER 2
    // CVREN (15): 1 = Comparator voltage reference circuit powered on
    // CVROE (14): 0 = CVREF voltage level is disconnected from CVREF20 pin
    // CVRSS (11): 0 = Comparator reference source CVRSRC = AVDD-AVSS
    // VREFSEL (10): 0 = Reference source for inverting input is from CVR1,
    //                   when CVR1CONbits.VREFSEL = 0
    // Unimplemented <13,12,9,8,7>: Read as ?0?
    // CVR(6-0): Comparator Voltage Reference Value Selection bits
    // CVREF = (CVR<6:0> / 128)* CVRSRC
 //   CVR2CON = 0x0000;

    // CM4FLTR: COMPARATOR 4 FILTER CONTROL REGISTER
    // Unimplemented(15-7): Read as ?0?
    // CFSEL(6-4): 0b000 = Comparator Filter Input Clock is FP
    // CFLTREN(3): 1 = Comparator Digital Filter is enabled
    // CFDIV(2-0): 111 Comparator Filter Clock Divide 1:128
    CM4FLTR = 0x000F;

    // Op-AMP 1,3,5,2 for Signal Conditioning Currents &
    // Comparator 4 for Fault Generation
    
    // CMxCON: COMPARATOR x CONTROL REGISTER (x = 1, 2, 3 OR 5)
    // CON(15): Op Amp/Comparator Enable bit
    // COE(14): Comparator Output Enable bit
    //          Not applicable,when configured as Op-AMP
    // CPOL(13): Comparator Output Polarity Select bit
    //           Not applicable when configured as Op-AMP
    // OPAEN(10):1 = Op Amp Enable bit
    // CEVT(9): Comparator Event bit - Not applicable when configured as Op-AMP
    // COUT(8): Comparator Output bit - Not applicable when configured as Op-AMP
    // EVPOL(7-6): Trigger/Event/Interrupt Polarity Select bits
    //            Not applicable when configuring as Op-AMP
    // CREF(4): Comparator Reference Select bit (VIN+ input)
    // CCH(1-0): Op Amp/Comparator Channel Select bits
    // Unimplemented(3-2,5,12-11): Read as ?0?

    // OP-AMP/COMPARATOR 1 CONTROL REGISTER
    // Not used in this application
    CM1CON = 0x0000;            // 0b000U U000 00U0 UU00 ;

    // OP-AMP/COMPARATOR 2 CONTROL REGISTER
    // Not used in this application
    CM2CON = 0x0000;            // 0b000U U000 00U0 UU00 ;
    // OP-AMP/COMPARATOR 3 CONTROL REGISTER
    // Not used in this application
    CM1CON = 0x0000;            // 0b000U U000 00U0 UU00 ;

    // COMPARATOR 4 CONTROL REGISTER
    // Not used in this application
    CM4CON = 0x0000;            // 0b000U U100 00U0 UU00
    // Enable OP-AMP/COMPARATOR MODULE
    // Disable OA2 Module 
    CM2CONbits.CON = 0;
    // Enable OA1 Module 
    CM1CONbits.CON = 0;
    // Enable OA3 Module 
    CM3CONbits.CON = 0;
    // Enable CMP4 Module 
    CM4CONbits.CON = 0;
#endif
    return;
}