
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
#include "Current.h"

TCalculatedCurrentValue CUR_tCurrentValue;
TCurrentParameter  CUR_tCurrentPara;

uint32_t CUR_ulPreTimePoint;
uint32_t CUR_ulCurrentTimePoint;
uint32_t CUR_ulTimeDiff;
void CUR_vCurrentCalculation(void)
{
	MC_ABC_T uiPhaseCurrent;
	uint16_t uiTempValue;

	uiPhaseCurrent.a = _Q15abs(MAM_tGetThreePhaseCurrent().a);
	uiPhaseCurrent.b = _Q15abs(MAM_tGetThreePhaseCurrent().b);
	uiPhaseCurrent.c = _Q15abs(MAM_tGetThreePhaseCurrent().c);

	if(uiPhaseCurrent.a > CUR_tCurrentPara.uiMaxPhaseAPeakValue)
	{
		CUR_tCurrentValue.uiPhaseAPeakValue = uiPhaseCurrent.a;
	}

	if(uiPhaseCurrent.b > CUR_tCurrentPara.uiMaxPhaseBPeakValue)
	{
		CUR_tCurrentValue.uiPhaseAPeakValue = uiPhaseCurrent.b;
	}

	if(uiPhaseCurrent.c > CUR_tCurrentPara.uiMaxPhaseCPeakValue)
	{
		CUR_tCurrentValue.uiPhaseAPeakValue = uiPhaseCurrent.c;
	}

	if(uiPhaseCurrent.a > uiPhaseCurrent.b)
	{
		uiTempValue = uiPhaseCurrent.a;
	}
	else
	{
		uiTempValue = uiPhaseCurrent.b;
	}

	if(uiTempValue > uiPhaseCurrent.c)
	{
		if(uiTempValue > CUR_tCurrentValue.uiPeakValue)
		{
			CUR_tCurrentValue.uiPeakValue =uiTempValue;
		}
	}
	else
	{
		uiTempValue = uiPhaseCurrent.c;
		if(uiTempValue > CUR_tCurrentValue.uiPeakValue)
		{
			CUR_tCurrentValue.uiPeakValue =uiTempValue;
		}
	}

	CUR_tCurrentValue.uiRMSValue = Q15SQRT(CUR_tCurrentValue.uiPeakValue);

}

void CUR_vCurrntPeakProtection(void)
{
	if((CUR_tCurrentValue.uiPeakValue > CUR_tCurrentPara.uiEnterSpeedThresholdValue)&&(CUR_tCurrentValue.uiPeakValue < CUR_tCurrentPara.uiMaxPeakValue))
	{
      //decrease speed, generate warning.
	}
	else
	{
		if(CUR_tCurrentValue.uiPeakValue < CUR_tCurrentPara.uiLeaveSpeedThresholdValue)
		{
			//recover to normal control.
		}
	}


	if(CUR_tCurrentValue.uiPeakValue > CUR_tCurrentPara.uiMaxPeakValue)
	{
		//cut off PWM output and generate error code.
	}

}


void CUR_vCurrentProtection(void)
{
	CUR_ulCurrentTimePoint = MAM_ulGetCurrentTimeTick();
	if(CUR_ulCurrentTimePoint > CUR_ulPreTimePoint)
	{
		CUR_ulTimeDiff = (CUR_ulCurrentTimePoint - CUR_ulPreTimePoint);
	}
	else
	{
		CUR_ulTimeDiff = (0xFFFF - CUR_ulPreTimePoint + CUR_ulCurrentTimePoint);
	}

	if(CUR_ulTimeDiff >= 2)//check every 20ms
	{
		CUR_vCurrentCalculation();

		CUR_vCurrntPeakProtection();
	}
}

TCalculatedCurrentValue CUR_tGetCurrentValue(void)
{
	return CUR_tCurrentValue;
}
