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
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=src/app/app_tasks/program_planner.c src/app/app_tasks/g_code.c src/app/app_tasks/motion.c src/app/app_tasks/nuts_bolts.c src/app/app_tasks/settings.c src/app/app_tasks/eeprom.c src/app/app_tasks/dee_emulation_pic32.c src/app/app_tasks/report.c src/app/app_tasks/stepper.c src/app/system_config/pic32_mx/system_init.c src/app/system_config/pic32_mx/system_interrupt.c src/app/system_config/pic32_mx/system_task.c src/app/main.c src/app/app.c src/bsp/cnc_rev_00/bsp_sys_init.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/app/app_tasks/program_planner.o ${OBJECTDIR}/src/app/app_tasks/g_code.o ${OBJECTDIR}/src/app/app_tasks/motion.o ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o ${OBJECTDIR}/src/app/app_tasks/settings.o ${OBJECTDIR}/src/app/app_tasks/eeprom.o ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o ${OBJECTDIR}/src/app/app_tasks/report.o ${OBJECTDIR}/src/app/app_tasks/stepper.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o ${OBJECTDIR}/src/app/main.o ${OBJECTDIR}/src/app/app.o ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o
POSSIBLE_DEPFILES=${OBJECTDIR}/src/app/app_tasks/program_planner.o.d ${OBJECTDIR}/src/app/app_tasks/g_code.o.d ${OBJECTDIR}/src/app/app_tasks/motion.o.d ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d ${OBJECTDIR}/src/app/app_tasks/settings.o.d ${OBJECTDIR}/src/app/app_tasks/eeprom.o.d ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d ${OBJECTDIR}/src/app/app_tasks/report.o.d ${OBJECTDIR}/src/app/app_tasks/stepper.o.d ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d ${OBJECTDIR}/src/app/main.o.d ${OBJECTDIR}/src/app/app.o.d ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/src/app/app_tasks/program_planner.o ${OBJECTDIR}/src/app/app_tasks/g_code.o ${OBJECTDIR}/src/app/app_tasks/motion.o ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o ${OBJECTDIR}/src/app/app_tasks/settings.o ${OBJECTDIR}/src/app/app_tasks/eeprom.o ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o ${OBJECTDIR}/src/app/app_tasks/report.o ${OBJECTDIR}/src/app/app_tasks/stepper.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o ${OBJECTDIR}/src/app/main.o ${OBJECTDIR}/src/app/app.o ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o

# Source Files
SOURCEFILES=src/app/app_tasks/program_planner.c src/app/app_tasks/g_code.c src/app/app_tasks/motion.c src/app/app_tasks/nuts_bolts.c src/app/app_tasks/settings.c src/app/app_tasks/eeprom.c src/app/app_tasks/dee_emulation_pic32.c src/app/app_tasks/report.c src/app/app_tasks/stepper.c src/app/system_config/pic32_mx/system_init.c src/app/system_config/pic32_mx/system_interrupt.c src/app/system_config/pic32_mx/system_task.c src/app/main.c src/app/app.c src/bsp/cnc_rev_00/bsp_sys_init.c


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/app/app_tasks/program_planner.o: src/app/app_tasks/program_planner.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/program_planner.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/program_planner.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/program_planner.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/program_planner.o.d" -o ${OBJECTDIR}/src/app/app_tasks/program_planner.o src/app/app_tasks/program_planner.c   
	
${OBJECTDIR}/src/app/app_tasks/g_code.o: src/app/app_tasks/g_code.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/g_code.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/g_code.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/g_code.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/g_code.o.d" -o ${OBJECTDIR}/src/app/app_tasks/g_code.o src/app/app_tasks/g_code.c   
	
${OBJECTDIR}/src/app/app_tasks/motion.o: src/app/app_tasks/motion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/motion.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/motion.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/motion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/motion.o.d" -o ${OBJECTDIR}/src/app/app_tasks/motion.o src/app/app_tasks/motion.c   
	
${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o: src/app/app_tasks/nuts_bolts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d" -o ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o src/app/app_tasks/nuts_bolts.c   
	
${OBJECTDIR}/src/app/app_tasks/settings.o: src/app/app_tasks/settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/settings.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/settings.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/settings.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/settings.o.d" -o ${OBJECTDIR}/src/app/app_tasks/settings.o src/app/app_tasks/settings.c   
	
${OBJECTDIR}/src/app/app_tasks/eeprom.o: src/app/app_tasks/eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/eeprom.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/eeprom.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/eeprom.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/eeprom.o.d" -o ${OBJECTDIR}/src/app/app_tasks/eeprom.o src/app/app_tasks/eeprom.c   
	
${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o: src/app/app_tasks/dee_emulation_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d" -o ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o src/app/app_tasks/dee_emulation_pic32.c   
	
${OBJECTDIR}/src/app/app_tasks/report.o: src/app/app_tasks/report.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/report.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/report.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/report.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/report.o.d" -o ${OBJECTDIR}/src/app/app_tasks/report.o src/app/app_tasks/report.c   
	
${OBJECTDIR}/src/app/app_tasks/stepper.o: src/app/app_tasks/stepper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/stepper.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/stepper.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/stepper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/stepper.o.d" -o ${OBJECTDIR}/src/app/app_tasks/stepper.o src/app/app_tasks/stepper.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o: src/app/system_config/pic32_mx/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o src/app/system_config/pic32_mx/system_init.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o: src/app/system_config/pic32_mx/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o src/app/system_config/pic32_mx/system_interrupt.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o: src/app/system_config/pic32_mx/system_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o src/app/system_config/pic32_mx/system_task.c   
	
${OBJECTDIR}/src/app/main.o: src/app/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app 
	@${RM} ${OBJECTDIR}/src/app/main.o.d 
	@${RM} ${OBJECTDIR}/src/app/main.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/main.o.d" -o ${OBJECTDIR}/src/app/main.o src/app/main.c   
	
${OBJECTDIR}/src/app/app.o: src/app/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app 
	@${RM} ${OBJECTDIR}/src/app/app.o.d 
	@${RM} ${OBJECTDIR}/src/app/app.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app.o.d" -o ${OBJECTDIR}/src/app/app.o src/app/app.c   
	
${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o: src/bsp/cnc_rev_00/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/bsp/cnc_rev_00 
	@${RM} ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d" -o ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o src/bsp/cnc_rev_00/bsp_sys_init.c   
	
else
${OBJECTDIR}/src/app/app_tasks/program_planner.o: src/app/app_tasks/program_planner.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/program_planner.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/program_planner.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/program_planner.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/program_planner.o.d" -o ${OBJECTDIR}/src/app/app_tasks/program_planner.o src/app/app_tasks/program_planner.c   
	
${OBJECTDIR}/src/app/app_tasks/g_code.o: src/app/app_tasks/g_code.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/g_code.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/g_code.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/g_code.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/g_code.o.d" -o ${OBJECTDIR}/src/app/app_tasks/g_code.o src/app/app_tasks/g_code.c   
	
${OBJECTDIR}/src/app/app_tasks/motion.o: src/app/app_tasks/motion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/motion.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/motion.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/motion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/motion.o.d" -o ${OBJECTDIR}/src/app/app_tasks/motion.o src/app/app_tasks/motion.c   
	
${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o: src/app/app_tasks/nuts_bolts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o.d" -o ${OBJECTDIR}/src/app/app_tasks/nuts_bolts.o src/app/app_tasks/nuts_bolts.c   
	
${OBJECTDIR}/src/app/app_tasks/settings.o: src/app/app_tasks/settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/settings.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/settings.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/settings.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/settings.o.d" -o ${OBJECTDIR}/src/app/app_tasks/settings.o src/app/app_tasks/settings.c   
	
${OBJECTDIR}/src/app/app_tasks/eeprom.o: src/app/app_tasks/eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/eeprom.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/eeprom.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/eeprom.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/eeprom.o.d" -o ${OBJECTDIR}/src/app/app_tasks/eeprom.o src/app/app_tasks/eeprom.c   
	
${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o: src/app/app_tasks/dee_emulation_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o.d" -o ${OBJECTDIR}/src/app/app_tasks/dee_emulation_pic32.o src/app/app_tasks/dee_emulation_pic32.c   
	
${OBJECTDIR}/src/app/app_tasks/report.o: src/app/app_tasks/report.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/report.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/report.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/report.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/report.o.d" -o ${OBJECTDIR}/src/app/app_tasks/report.o src/app/app_tasks/report.c   
	
${OBJECTDIR}/src/app/app_tasks/stepper.o: src/app/app_tasks/stepper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/app_tasks 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/stepper.o.d 
	@${RM} ${OBJECTDIR}/src/app/app_tasks/stepper.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app_tasks/stepper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app_tasks/stepper.o.d" -o ${OBJECTDIR}/src/app/app_tasks/stepper.o src/app/app_tasks/stepper.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o: src/app/system_config/pic32_mx/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_init.o src/app/system_config/pic32_mx/system_init.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o: src/app/system_config/pic32_mx/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_interrupt.o src/app/system_config/pic32_mx/system_interrupt.c   
	
${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o: src/app/system_config/pic32_mx/system_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app/system_config/pic32_mx 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d 
	@${RM} ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o.d" -o ${OBJECTDIR}/src/app/system_config/pic32_mx/system_task.o src/app/system_config/pic32_mx/system_task.c   
	
${OBJECTDIR}/src/app/main.o: src/app/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app 
	@${RM} ${OBJECTDIR}/src/app/main.o.d 
	@${RM} ${OBJECTDIR}/src/app/main.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/main.o.d" -o ${OBJECTDIR}/src/app/main.o src/app/main.c   
	
${OBJECTDIR}/src/app/app.o: src/app/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/app 
	@${RM} ${OBJECTDIR}/src/app/app.o.d 
	@${RM} ${OBJECTDIR}/src/app/app.o 
	@${FIXDEPS} "${OBJECTDIR}/src/app/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/app/app.o.d" -o ${OBJECTDIR}/src/app/app.o src/app/app.c   
	
${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o: src/bsp/cnc_rev_00/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/bsp/cnc_rev_00 
	@${RM} ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"src" -I"src/app" -I"src/bsp" -I"src/bsp/cnc_rev_00" -I"src/app/system_config" -I"src/app/system_config/pic32_mx" -I"../../../../../../microchip/xc32/v1.31/pic32-libs/peripheral" -I"src/app/app_tasks" -MMD -MF "${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o.d" -o ${OBJECTDIR}/src/bsp/cnc_rev_00/bsp_sys_init.o src/bsp/cnc_rev_00/bsp_sys_init.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/xc32/v1.31/pic32mx/lib/libmchp_peripheral_32MX795F512L.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\xc32\v1.31\pic32mx\lib\libmchp_peripheral_32MX795F512L.a       -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/xc32/v1.31/pic32mx/lib/libmchp_peripheral_32MX795F512L.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\xc32\v1.31\pic32mx\lib\libmchp_peripheral_32MX795F512L.a      -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/CNC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
