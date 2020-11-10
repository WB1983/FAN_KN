#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "hal/board_service.h"
#include "MainAlgorithom.h"

/*button test for the button*/
void vButtonTest(void)
{
    static uint8_t ucSwitchFlag = 0;

    if(IsPressed_Button1())
    {
        if(ucSwitchFlag == 0)
        {
            ucSwitchFlag = 1;  
            INRUSH_RELAY = 0;
        }
        else
        {
            ucSwitchFlag = 0;
            INRUSH_RELAY = 1;
        }
        
    }
        
}
