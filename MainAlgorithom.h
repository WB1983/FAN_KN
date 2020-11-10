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
    
/*********************************definition*******************************/
#define MAM_STOP_SPEED  300
 typedef struct _TBrakeParameter
 {
     uint16_t ulDecreaseStep;
     uint16_t ulStopSpeed;
 }TBrakeParameter;
 
  typedef struct _TStartParameter
 {
     uint16_t ulAlignIQValue;
     uint16_t ulAlignTime;
     uint16_t ulRampupStep;
     uint16_t ulRampupKeepTime;
 }TStartParameter;
 
typedef struct _TControlParameter
{
    TBrakeParameter tBreakPara;
    TStartParameter tStartPara;
}TControlParameter;

typedef struct _TControlData
{
    uint16_t ulSystemState;
    uint16_t ulTargetSpeed;
    
    uint16_t ulRampupDelay;
}TControlData;


enum SYS_STATE
{
    SYS_IDLE,
    SYS_RUN,
    SYS_BRAKE
};

#define BRAKE_INIT_PARA {1, ENDSPEED_ELECTR}

#define START_INIT_PARA {0,0,0,2000}

#define CONTROL_INT_DATA {SYS_IDLE, 0, 0}

/*******************************function claraficaion********************/
    //first initialization
extern void MAM_vInitializationOnce(void);

//first initialization
extern void MAM_vInitialization2Once(void);

//Feed external watchdog
extern void MAM_vFeedExternalWatchdog(void);

//Control motor on/off
extern void MAM_vMotorControl(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MAINALGRITHOM_H */

