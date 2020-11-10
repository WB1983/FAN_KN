#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "hal/board_service.h"
#include "MainAlgorithom.h"
#include  "MMI.h"

int main ( void )
{
    
    //first initialization!
    MAM_vInitializationOnce();
    
    //another initialization
    MAM_vInitialization2Once();
    
    MAM_vApplicationInitialization();
            
    while(1)
    {
        MAM_vFeedExternalWatchdog();

        //DiagnosticsStepMain();
        BoardService();

        //vButtonTest();
        MAM_vMotorControl();
        
        MAM_vMotorSpeedAdjustment();
        //vButtonTest();
        
        

    }//inner while loop

}// End of Main loop
