#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "hal/board_service.h"
#include "MainAlgorithom.h"

int main ( void )
{
    
    //first initialization!
    MAM_vInitializationOnce();
    
    //another initialization
    MAM_vInitialization2Once();
            
    while(1)
    {
        MAM_vFeedExternalWatchdog();

        //DiagnosticsStepMain();
        BoardService();

        //vButtonTest();
        MAM_vMotorControl();

    }//inner while loop

}// End of Main loop
