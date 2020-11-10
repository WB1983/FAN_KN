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
#include "Power.h"
#include "MainAlgorithom.h"

static uint32_t POW_ulPower;
static uint32_t POW_ulPreTimePoint;
static uint32_t POW_ulCurrentTimePoint;
static uint32_t POW_ulTimeDiff;

void POW_vPowerCalculation(void)
{
	uint32_t uiTemp;
	uiTemp = NOPOLESPAIRS*LRR_BACK_EMF*CUR_tGetCurrentValue().uiRMSValue;
	uiTemp = LRR_CUR_PHASE_NR*(uiTemp/60)*1000/MAM_uiGetCurrentSpeed();
	POW_ulPower = uiTemp;
}

void POW_vPowerProtection(void)
	{
		if(POW_ulPower > POW_PEAK_LIMIT)
		{
			//cut off the PWM and generate error or limit power
		}
		else
		{
			if(POW_ulPower > POW_ENTRY_LIMIT)
			{
				//Decrease speed
			}
			else
			{
				if(POW_ulPower < POW_LEAVE_LIMIT)
				{
					//revover to normal
				}
			}
		}
	}

void POW_vPowerCheck(void)
{
	POW_ulCurrentTimePoint = MAM_ulGetCurrentTimeTick();
	if(POW_ulCurrentTimePoint > POW_ulPreTimePoint)
	{
		POW_ulTimeDiff = (POW_ulCurrentTimePoint - POW_ulPreTimePoint);
	}
	else
	{
		POW_ulTimeDiff = (0xFFFF - POW_ulPreTimePoint + POW_ulCurrentTimePoint);
	}

	if(POW_ulTimeDiff >= 4)//check every 20ms
	{
		POW_vPowerCalculation();

		POW_vPowerProtection();
	}
}
