/* 
 * File:   LockRotor.h
 * Author: WANG
 *
 * Created on October 22, 2020, 12:56 AM
 */

#ifndef LOCKROTOR_H
#define	LOCKROTOR_H

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
    
typedef struct _TBackEmf
{
    uint16_t Ealpha;
    uint16_t Ebelta;
}TBackEmf;

typedef struct _TLockRotor
{
    uint16_t BackEMF;
    uint16_t BackEMFFactor;
}TLockRotor;

#define LRR_MIN_BACK_EMF           20

#define LRR_BACK_EMF               26

#define LRR_CUR_PHASE_NR           3
#ifdef	__cplusplus
}
#endif

#endif	/* LOCKROTOR_H */

