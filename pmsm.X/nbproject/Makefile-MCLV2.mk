#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-MCLV2.mk)" "nbproject/Makefile-local-MCLV2.mk"
include nbproject/Makefile-local-MCLV2.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=MCLV2
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../hal/uart1.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/port_config.c ../hal/adc.c ../hal/pwm.c ../meascurr.s ../readadc0.s ../q15sqrt.s ../atan2CORDIC.s ../Fdweak.c ../smcpos.c ../main.c ../MMI.c ../MainAlgorithom.c C:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/adc.o ${OBJECTDIR}/_ext/1360926148/pwm.o ${OBJECTDIR}/_ext/1472/meascurr.o ${OBJECTDIR}/_ext/1472/readadc0.o ${OBJECTDIR}/_ext/1472/q15sqrt.o ${OBJECTDIR}/_ext/1472/atan2CORDIC.o ${OBJECTDIR}/_ext/1472/Fdweak.o ${OBJECTDIR}/_ext/1472/smcpos.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/MMI.o ${OBJECTDIR}/_ext/1472/MainAlgorithom.o ${OBJECTDIR}/_ext/1778585070/X2CScope.o ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360926148/uart1.o.d ${OBJECTDIR}/_ext/1360926148/board_service.o.d ${OBJECTDIR}/_ext/1360926148/clock.o.d ${OBJECTDIR}/_ext/1360926148/device_config.o.d ${OBJECTDIR}/_ext/1360926148/port_config.o.d ${OBJECTDIR}/_ext/1360926148/adc.o.d ${OBJECTDIR}/_ext/1360926148/pwm.o.d ${OBJECTDIR}/_ext/1472/meascurr.o.d ${OBJECTDIR}/_ext/1472/readadc0.o.d ${OBJECTDIR}/_ext/1472/q15sqrt.o.d ${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d ${OBJECTDIR}/_ext/1472/Fdweak.o.d ${OBJECTDIR}/_ext/1472/smcpos.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/MMI.o.d ${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d ${OBJECTDIR}/_ext/1778585070/X2CScope.o.d ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/adc.o ${OBJECTDIR}/_ext/1360926148/pwm.o ${OBJECTDIR}/_ext/1472/meascurr.o ${OBJECTDIR}/_ext/1472/readadc0.o ${OBJECTDIR}/_ext/1472/q15sqrt.o ${OBJECTDIR}/_ext/1472/atan2CORDIC.o ${OBJECTDIR}/_ext/1472/Fdweak.o ${OBJECTDIR}/_ext/1472/smcpos.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/MMI.o ${OBJECTDIR}/_ext/1472/MainAlgorithom.o ${OBJECTDIR}/_ext/1778585070/X2CScope.o ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o

# Source Files
SOURCEFILES=../hal/uart1.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/port_config.c ../hal/adc.c ../hal/pwm.c ../meascurr.s ../readadc0.s ../q15sqrt.s ../atan2CORDIC.s ../Fdweak.c ../smcpos.c ../main.c ../MMI.c ../MainAlgorithom.c C:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-MCLV2.mk dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP128MC506
MP_LINKER_FILE_OPTION=,--script=p33EP128MC506.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/uart1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/board_service.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/clock.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/device_config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/port_config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/adc.o: ../hal/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/adc.c  -o ${OBJECTDIR}/_ext/1360926148/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/pwm.o: ../hal/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/pwm.c  -o ${OBJECTDIR}/_ext/1360926148/pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/pwm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/Fdweak.o: ../Fdweak.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Fdweak.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Fdweak.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Fdweak.c  -o ${OBJECTDIR}/_ext/1472/Fdweak.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Fdweak.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Fdweak.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/smcpos.o: ../smcpos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/smcpos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smcpos.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../smcpos.c  -o ${OBJECTDIR}/_ext/1472/smcpos.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/smcpos.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smcpos.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/MMI.o: ../MMI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/MMI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/MMI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MMI.c  -o ${OBJECTDIR}/_ext/1472/MMI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/MMI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/MMI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/MainAlgorithom.o: ../MainAlgorithom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/MainAlgorithom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MainAlgorithom.c  -o ${OBJECTDIR}/_ext/1472/MainAlgorithom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1778585070/X2CScope.o: C\:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1778585070" 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScope.o.d 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScope.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c  -o ${OBJECTDIR}/_ext/1778585070/X2CScope.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1778585070/X2CScope.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1778585070/X2CScope.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o: C\:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1778585070" 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c  -o ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/uart1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/board_service.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/clock.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/device_config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/port_config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/adc.o: ../hal/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/adc.c  -o ${OBJECTDIR}/_ext/1360926148/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/adc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360926148/pwm.o: ../hal/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/pwm.c  -o ${OBJECTDIR}/_ext/1360926148/pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360926148/pwm.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360926148/pwm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/Fdweak.o: ../Fdweak.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/Fdweak.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Fdweak.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Fdweak.c  -o ${OBJECTDIR}/_ext/1472/Fdweak.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/Fdweak.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Fdweak.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/smcpos.o: ../smcpos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/smcpos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smcpos.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../smcpos.c  -o ${OBJECTDIR}/_ext/1472/smcpos.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/smcpos.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smcpos.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/MMI.o: ../MMI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/MMI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/MMI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MMI.c  -o ${OBJECTDIR}/_ext/1472/MMI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/MMI.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/MMI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/MainAlgorithom.o: ../MainAlgorithom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/MainAlgorithom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MainAlgorithom.c  -o ${OBJECTDIR}/_ext/1472/MainAlgorithom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/MainAlgorithom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1778585070/X2CScope.o: C\:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1778585070" 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScope.o.d 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScope.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/WANG/KN_FAN/FAN_KN/X2CScope.c  -o ${OBJECTDIR}/_ext/1778585070/X2CScope.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1778585070/X2CScope.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1778585070/X2CScope.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o: C\:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1778585070" 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d 
	@${RM} ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeCommunication.c  -o ${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -O0 -I".." -I"../lib" -I"../diagnostics" -DMCLV2 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1778585070/X2CScopeCommunication.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/meascurr.o: ../meascurr.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/meascurr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/meascurr.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../meascurr.s  -o ${OBJECTDIR}/_ext/1472/meascurr.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/meascurr.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/meascurr.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/readadc0.o: ../readadc0.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/readadc0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/readadc0.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../readadc0.s  -o ${OBJECTDIR}/_ext/1472/readadc0.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/readadc0.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/readadc0.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/q15sqrt.o: ../q15sqrt.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/q15sqrt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/q15sqrt.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../q15sqrt.s  -o ${OBJECTDIR}/_ext/1472/q15sqrt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/q15sqrt.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/q15sqrt.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/atan2CORDIC.o: ../atan2CORDIC.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/atan2CORDIC.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../atan2CORDIC.s  -o ${OBJECTDIR}/_ext/1472/atan2CORDIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1472/meascurr.o: ../meascurr.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/meascurr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/meascurr.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../meascurr.s  -o ${OBJECTDIR}/_ext/1472/meascurr.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/meascurr.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/meascurr.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/readadc0.o: ../readadc0.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/readadc0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/readadc0.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../readadc0.s  -o ${OBJECTDIR}/_ext/1472/readadc0.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/readadc0.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/readadc0.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/q15sqrt.o: ../q15sqrt.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/q15sqrt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/q15sqrt.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../q15sqrt.s  -o ${OBJECTDIR}/_ext/1472/q15sqrt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/q15sqrt.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/q15sqrt.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1472/atan2CORDIC.o: ../atan2CORDIC.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/atan2CORDIC.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../atan2CORDIC.s  -o ${OBJECTDIR}/_ext/1472/atan2CORDIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  -Wall  -I"../" -Wa,-MD,"${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/atan2CORDIC.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../lib/motor_control/libmotor_control_dspic33e-elf.a C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeLib_dsPIC33EP.X.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\lib\motor_control\libmotor_control_dspic33e-elf.a C:\Users\WANG\KN_FAN\FAN_KN\X2CScopeLib_dsPIC33EP.X.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=q,--no-force-link,--smart-io,--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../lib/motor_control/libmotor_control_dspic33e-elf.a C:/Users/WANG/KN_FAN/FAN_KN/X2CScopeLib_dsPIC33EP.X.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\lib\motor_control\libmotor_control_dspic33e-elf.a C:\Users\WANG\KN_FAN\FAN_KN\X2CScopeLib_dsPIC33EP.X.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_MCLV2=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wall  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=q,--no-force-link,--smart-io,--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/pmsm.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/MCLV2
	${RM} -r dist/MCLV2

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
