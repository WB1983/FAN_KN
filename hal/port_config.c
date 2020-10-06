/*******************************************************************************
  Input / Output Port Configuration Routine source File

  File Name:
    port_config.c

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
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "pim_select.h"

#undef MCBOARD_UART
// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void MapGPIOHWFunction (void);
// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
#ifdef INTERNAL_OPAMP_PIM

    //Ia Out : 42
    TRISAbits.TRISA0 = 1;   // PIN13:AN0/OA2OUT/RA0
	ANSELAbits.ANSA0 = 1;	// RA0 (AN0) Opamp output OA2OUT used for Ia

    //Ia+ PIM : 74
    TRISAbits.TRISA1 = 1;   // PIN14:AN1/C2IN1+/RA1
    ANSELAbits.ANSA1 = 1;	// RA1 (AN1) Opamp input C2IN1+ used for Ia

    //Ia- PIM : - (66)
    TRISBbits.TRISB0 = 1;   // PIN15:PGED3/VREF-/AN2/C2IN1-/SS1/RPI32/CTED2/RB0
    ANSELBbits.ANSB0 = 1;	// RB0 (AN2) Opamp input C2IN1- used for Ia

    //Ib Out : 58
    TRISCbits.TRISC0 = 1;   // PIN21:AN6/OA3OUT/C4IN1+/OCFB/RC0
    ANSELCbits.ANSC0 = 1;	// RC0 (AN6) Opamp output C3OUT used for Ib

    //Ib+ PIM: 73
    TRISCbits.TRISC2 = 1;   // PIN23:AN8/C3IN1+/U1RTS/BCLK1/FLT3/RC2
    ANSELCbits.ANSC2 = 1;	// RC2 (AN8) Opamp input C3IN1+ used for Ib

    //Ib- PIM: - (66)
    TRISCbits.TRISC1 = 1;   // PIN22:AN7/C3IN1-/C4IN1-/RC1
    ANSELCbits.ANSC1 = 1;	// RC1 (AN7) Opamp input C3IN1- used for Ib

    //Isum Out : 59
    TRISBbits.TRISB1 = 1;   // PIN16:PGEC3/VREF+/AN3/OA1OUT/RPI33/CTED1/RB1
    ANSELBbits.ANSB1 = 1;	// RB1 (AN3) Opamp output OA1OUT used for Isum

    //Isum+ PIM: 66
    TRISBbits.TRISB2 = 1;   // PIN17:PGEC1/AN4/C1IN1+/RPI34/RB2
    ANSELBbits.ANSB2 = 1;	// RB2 (AN4) Opamp input C1IN1+ used for Isum

    //Isum- PIM: 67
    TRISBbits.TRISB3 = 1;   // PIN18:PGED1/AN5/C1IN1-/RP35/RB3
    ANSELBbits.ANSB3 = 1;	// RB3 (AN5) Opamp input C1IN1- used for Isum

    // FAULT Pins
    // FAULT : PIM #18
    TRISBbits.TRISB4 = 1;            // FLT32/SCL2/RP36/RB4
    RPINR12bits.FLT1R = 32;          // Make Pin RP32 FLT I/P    

#else

    // IPHASE2_A/IB_A(AN0) : PIM #42
    TRISAbits.TRISA0 = 1;    // AN0/OA2OUT/RA0
    ANSELAbits.ANSA0 = 1;

    // IPHASE1_A/IA_A(AN1) : PIM #41
    TRISAbits.TRISA1 = 1;    // AN1/C2IN1+/RA1
    ANSELAbits.ANSA1 = 1;

    // FAULT Pins
    // FAULT : PIM #18
    TRISBbits.TRISB4 = 1;            // FLT32/SCL2/RP36/RB4
    RPINR12bits.FLT1R = 32;          // Make Pin RP32 FLT I/P

#endif

    // Potentiometer #1 input - used as Speed Reference
    // POT1 : PIM #32
    TRISEbits.TRISE13 = 1;          // AN13/C3IN2-/U2CTS/RE13
    ANSELEbits.ANSE13 = 1;

    // Inverter DC bus voltage Sensing
    // VBUS_A : PIM #35
    TRISAbits.TRISA12 = 1;          // AN10/RPI28/RA12
    ANSELAbits.ANSA12 = 1;

    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : PIM #93  (RPI47/PWM1L1/T5CK/RB15)
    // PWM1H : PIM #94  (RPI46/PWM1H1/T3CK/RB14)
    // PWM2L : PIM #98  (RP45/PWM1L2/CTPLS/RB13)
    // PWM2H : PIM #99  (RP44/PWM1H2/RB12)
    // PWM3L : PIM #100 (RP43/PWM1L3/RB11)
    // PWM3H : PIM #03  (RP42/PWM1H3/RB10)
    TRISBbits.TRISB14 = 0 ;
    TRISBbits.TRISB15 = 0 ;
    TRISBbits.TRISB12 = 0 ;
    TRISBbits.TRISB13 = 0 ;
    TRISBbits.TRISB10 = 0 ;
    TRISBbits.TRISB11 = 0 ;

    // Debug LEDs
    // LED1 : PIM #01
    TRISDbits.TRISD5 = 0;           // AN56/RA10
    // LED2 : PIM #60
    TRISDbits.TRISD6 = 0;           // RPI72/RD8

    // Push button Switches
#ifdef MCLV2
    // SW1 : PIM #83
    TRISGbits.TRISG7 = 1;            // AN30/CVREF+/RPI52/RC4
    // SW2 : PIM #84
    TRISGbits.TRISG6 = 1;            // AN19/RP118/RG6
#endif

#ifdef MCHV2_MCHV3
    // Push Button : PIM #68
    TRISBbits.TRISB8 = 1;   // PIN48: RB8
#endif

    // UART - for RTDM/DMCI Communication
    // UART_RX : PIM #49 (Input)
    TRISCbits.TRISC5 = 1;            // SCL1/RPI53/RC5
    // UART_TX : PIM #50(Output)
    TRISFbits.TRISF1 = 0;            // RP97/RF1

    /************************** Re-mappable Pin Configuration ******************/

    //Unlock registers by clearing IOLOCK bit OSCCON(OSCONbits.IOLOCK = 0)
    __builtin_write_OSCCONL(OSCCON & (~(1 << 6))); 

    // X2C Communication RX and TX configuration ( UART #1)
    // UART_RX : PIM #49 (Input)
    // Configure RP53 as U1RX
    _U1RXR = 53;         // SCL1/RPI53/RC5
    // UART_TX : PIM #50 (Output)
    // Remap RP53 as U1RX
    _RP97R = 0x01;                  // RP97/RF1

    // Lock registers by setting IOLOCK bit OSCCON(OSCONbits.IOLOCK = 1)
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Set bit 6

    /**************************************************************************/
    return;
}