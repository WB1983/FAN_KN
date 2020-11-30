#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "hal/board_service.h"
#include "MainAlgorithom.h"
#include  "MMI.h"
#include "X2CScopeCommunication.h"
#include "X2CScope.h"
static uint16_t MAM_uiVbus = 0;
static uint16_t MAM_uiPhaseA = 0;
static uint16_t MAM_uiPhaseB = 0;
static uint16_t MAM_uiPhaseC = 0;

int main ( void )
{
    
    //first initialization!
    MAM_vInitializationOnce();
    
    //another initialization
    MAM_vInitialization2Once();
    
    MAM_vApplicationInitialization();
            
    while(1)
    {
        MAM_uiVbus = ADCBUF_SPEED_REF_A;
        MAM_uiPhaseA = ADCBUF_INV_A_IPHASE1;
        MAM_uiPhaseB = ADCBUF_INV_A_IPHASE2;
        
        MAM_10MSTimer();
        
        MAM_vFeedExternalWatchdog();

//        DiagnosticsStepMain();
        BoardService();

        //vButtonTest();
        MAM_vMotorControl();
        
        MAM_vMotorSpeedAdjustment();
        //vButtonTest();
        
        X2CScope_Communicate();

    }//inner while loop

}// End of Main loop
