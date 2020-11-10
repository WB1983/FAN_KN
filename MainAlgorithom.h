/* 
 * File:   MainAlgrithom.h
 * Author: WANG
 *
 * Created on October 10, 2020, 3:29 PM
 */

#ifndef MAINALGRITHOM_H
#define	MAINALGRITHOM_H

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

 #include"LockRotor.h"   
    
/*********************************definition*******************************/
#define MAM_STOP_SPEED  300

typedef struct _TSlopingData
{
    uint16_t uiMaxValue;//slop max value
    uint16_t uiMinValue;//slop min value
    uint16_t uiStep;//step
    uint16_t uiOutputValue;//output value
}TSlopingData;

 typedef struct _TBrakeParameter
 {
     uint16_t uiDecreaseStep;//slow speed rate
     uint16_t uiStopSpeed;//stop target speed
 }TBrakeParameter;
 
 typedef struct _TAlignSlopParameter
 {
     uint16_t uiAlignTime;//ms
     TSlopingData tAlignSlop;//align slop
 }TAlignSlopParameter;

 typedef struct _TRampupSlopParameter
 {
	 uint16_t uiRampupTime;
	 TSlopingData tRampupSlop;
 }TRampupSlopParameter;

  typedef struct _TStartParameter
 {
     TAlignSlopParameter tAlignSlopParameter;
     TRampupSlopParameter tRampupSlopParameter;
 }TStartParameter;
 
 typedef struct _TSpeedControlParameter
 {
 	uint16_t uiMaxSpeed;//system max speed
 	uint16_t uiMinSpeed;//system min speed
 }TSpeedControlParameter;


typedef struct _TControlParameter
{
    TBrakeParameter tBreakPara;//braking para
    TStartParameter tStartPara;//start para
    TSpeedControlParameter tSpeedControlParameter;//speed control parameter
    
}TControlParameter;

typedef struct _TControlData
{
    uint16_t uiSystemState;//system state
    uint16_t uiTargetSpeed;//target speed
    uint8_t  uiDescendAscend;//speed up or down
}TControlData;

typedef struct _TTorqueControlPara
{
    uint16_t uiMaxTorque;//system max speed
 	uint16_t uiMinTorque;//system min speed
    uint16_t uiTorqueRampupStep;
    uint16_t uiTorqueRampDowmStep;
}TTorqueControlPara;

typedef struct _TTorqueControlData
{
    int16_t iTorqueTempCtrlValue;
    int16_t iTorqueDiffValue;
    uint16_t uiTorqueRampCount;
    uint16_t uiTorqueTargetValue;
}TTorqueControlData;

#define TORQUE_CONTROL_PARA_DEFAULT {NORM_CURRENT(2.0),NORM_CURRENT(1.0),1,1}
#define TORQUE_CONTROL_DATA_DEFAULT {0,0,0,NORM_CURRENT(1.6)}
enum SYS_STATE
{
    SYS_IDLE,
    SYS_RUN,
    SYS_BRAKE
};

enum DIRECTION
{
    ASCENDING,
    DESCENDING
};

#define ALIGN_TIME 500//milisecond

#define ALIGN_REF_FACTOR     5

#define ALIGN_STEP Q_CURRENT_REF_OPENLOOP/(ALIGN_TIME/(ALIGN_REF_FACTOR*LOOPTIME_SEC*1000))

#define ALIGN_SLOP_DEFAULT {Q_CURRENT_REF_OPENLOOP_MAX,Q_CURRENT_REF_OPENLOOP_MIN,ALIGN_STEP,0}

#define BRAKE_INIT_PARA {SPEEDREFRAMP*2, ENDSPEED_ELECTR}

#define START_INIT_PARA {{ALIGN_TIME,ALIGN_SLOP_DEFAULT},{0,{0,0,0,0}}}

#define SPD_CONTROL_PARA {MAXIMUM_SPEED_RPM*NOPOLESPAIRS, END_SPEED_RPM*NOPOLESPAIRS}

#define CONTROL_INT_DATA {SYS_IDLE, 0, DESCENDING}

#define OPL_RAMP_LIST_NR 12

#define TORQUEREFRAMP_COUNT 200


//#define DELAY_TIME 200//Milisecond

//#define DELAY_TIME_COUNT DELAY_TIME/(LOOPTIME_SEC*1000)


#define DEBUG_SPEED_STEP      10
#define DEBUG_SPEED_STEP_ELEC DEBUG_SPEED_STEP*NOPOLESPAIRS

/*******************************function claraficaion********************/
    //first initialization
extern void MAM_vInitializationOnce(void);

//first initialization
extern void MAM_vInitialization2Once(void);

//Feed external watchdog
extern void MAM_vFeedExternalWatchdog(void);

//Control motor on/off
extern void MAM_vMotorControl(void);

//get three phase current
extern MC_ABC_T MAM_tGetThreePhaseCurrent(void);

extern uint16_t MAM_uiGetCurrentSpeed(void);

extern void MAM_vSetTargetSpeed(uint16_t uiTargetSpeed);

extern MC_ABC_T MAM_tGetThreePhaseCurrent(void);

extern uint32_t MAM_ulGetCurrentTimeTick(void);

extern TBackEmf MAM_tGetBackEmf(void);

extern void MAM_vMotorSpeedAdjustment(void);

extern void MAM_vApplicationInitialization(void);

extern void MAM_vSetTargetTorque(uint16_t uiTargetTorque);
#ifdef	__cplusplus
}
#endif

#endif	/* MAINALGRITHOM_H */

