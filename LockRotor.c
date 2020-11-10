#ifdef __XC16__  // See comments at the top of this header file
#include <xc.h>
#endif // __XC16__
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "smcpos.h"
#include "LockRotor.h"
#include "Current.h"

#include "MainAlgorithom.h"
static TLockRotor LRR_tTLockRotor;
static TBackEmf LRR_tBackEmf;

void LRR_vLockRotorCheck(void)
{
	LRR_tBackEmf = MAM_tGetBackEmf();

	LRR_tTLockRotor.BackEMF = Q15SQRT(LRR_tBackEmf.Ealpha*LRR_tBackEmf.Ealpha + LRR_tBackEmf.Ebelta*LRR_tBackEmf.Ebelta);
	LRR_tTLockRotor.BackEMFFactor= LRR_tTLockRotor.BackEMF/MAM_uiGetCurrentSpeed();

    if(LRR_tTLockRotor.BackEMFFactor < LRR_MIN_BACK_EMF)
    {
    	//cut off the PWM and generate error.
    }

}
