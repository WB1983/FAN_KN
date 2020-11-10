/* 
 * File:   Current.h
 * Author: WANG
 *
 * Created on October 22, 2020, 12:55 AM
 */

#ifndef Current_H
#define	Current_H

#ifdef	__cplusplus
extern "C" {
#endif

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
    
#include "MainAlgorithom.h"
 
typedef struct _TCalculatedCurrentValue
{
    uint16_t uiRMSValue;
    uint16_t uiPeakValue;
    uint16_t uiPhaseAPeakValue;
    uint16_t uiPhaseBPeakValue;
    uint16_t uiPhaseCPeakValue;
}TCalculatedCurrentValue;

typedef struct _TCurrentParameter
{
	uint16_t uiMaxRmsValue;
	uint16_t uiMaxOffsetValue;
	uint16_t uiMaxPeakValue;
	uint16_t uiEnterSpeedThresholdValue;
	uint16_t uiLeaveSpeedThresholdValue;
	uint16_t uiMaxPhaseAPeakValue;
	uint16_t uiMaxPhaseBPeakValue;
	uint16_t uiMaxPhaseCPeakValue;
}TCurrentParameter;

extern TCalculatedCurrentValue CUR_tGetCurrentValue(void);
#ifdef	__cplusplus
}
#endif

#endif	/* POWER_H */

