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
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=delays.asm main.asm mcp23s17.asm ssp_hw.asm databus.asm addressbus.asm usart_hw.asm usart_sw.asm rom.asm

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/delays.o ${OBJECTDIR}/main.o ${OBJECTDIR}/mcp23s17.o ${OBJECTDIR}/ssp_hw.o ${OBJECTDIR}/databus.o ${OBJECTDIR}/addressbus.o ${OBJECTDIR}/usart_hw.o ${OBJECTDIR}/usart_sw.o ${OBJECTDIR}/rom.o
POSSIBLE_DEPFILES=${OBJECTDIR}/delays.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/mcp23s17.o.d ${OBJECTDIR}/ssp_hw.o.d ${OBJECTDIR}/databus.o.d ${OBJECTDIR}/addressbus.o.d ${OBJECTDIR}/usart_hw.o.d ${OBJECTDIR}/usart_sw.o.d ${OBJECTDIR}/rom.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/delays.o ${OBJECTDIR}/main.o ${OBJECTDIR}/mcp23s17.o ${OBJECTDIR}/ssp_hw.o ${OBJECTDIR}/databus.o ${OBJECTDIR}/addressbus.o ${OBJECTDIR}/usart_hw.o ${OBJECTDIR}/usart_sw.o ${OBJECTDIR}/rom.o

# Source Files
SOURCEFILES=delays.asm main.asm mcp23s17.asm ssp_hw.asm databus.asm addressbus.asm usart_hw.asm usart_sw.asm rom.asm


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18f452
MP_LINKER_DEBUG_OPTION=-r=ROM@0x7DC0:0x7FFF -r=RAM@GPR:0x5F4:0x5FF
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/delays.o: delays.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/delays.o.d 
	@${RM} ${OBJECTDIR}/delays.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/delays.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/delays.lst\\\" -e\\\"${OBJECTDIR}/delays.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/delays.o\\\" \\\"delays.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/delays.o"
	@${FIXDEPS} "${OBJECTDIR}/delays.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/main.o: main.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/main.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/main.lst\\\" -e\\\"${OBJECTDIR}/main.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/main.o\\\" \\\"main.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/main.o"
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/mcp23s17.o: mcp23s17.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/mcp23s17.o.d 
	@${RM} ${OBJECTDIR}/mcp23s17.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/mcp23s17.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/mcp23s17.lst\\\" -e\\\"${OBJECTDIR}/mcp23s17.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/mcp23s17.o\\\" \\\"mcp23s17.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/mcp23s17.o"
	@${FIXDEPS} "${OBJECTDIR}/mcp23s17.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/ssp_hw.o: ssp_hw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ssp_hw.o.d 
	@${RM} ${OBJECTDIR}/ssp_hw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/ssp_hw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/ssp_hw.lst\\\" -e\\\"${OBJECTDIR}/ssp_hw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/ssp_hw.o\\\" \\\"ssp_hw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/ssp_hw.o"
	@${FIXDEPS} "${OBJECTDIR}/ssp_hw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/databus.o: databus.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/databus.o.d 
	@${RM} ${OBJECTDIR}/databus.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/databus.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/databus.lst\\\" -e\\\"${OBJECTDIR}/databus.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/databus.o\\\" \\\"databus.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/databus.o"
	@${FIXDEPS} "${OBJECTDIR}/databus.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/addressbus.o: addressbus.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/addressbus.o.d 
	@${RM} ${OBJECTDIR}/addressbus.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/addressbus.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/addressbus.lst\\\" -e\\\"${OBJECTDIR}/addressbus.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/addressbus.o\\\" \\\"addressbus.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/addressbus.o"
	@${FIXDEPS} "${OBJECTDIR}/addressbus.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/usart_hw.o: usart_hw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart_hw.o.d 
	@${RM} ${OBJECTDIR}/usart_hw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/usart_hw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/usart_hw.lst\\\" -e\\\"${OBJECTDIR}/usart_hw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/usart_hw.o\\\" \\\"usart_hw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/usart_hw.o"
	@${FIXDEPS} "${OBJECTDIR}/usart_hw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/usart_sw.o: usart_sw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart_sw.o.d 
	@${RM} ${OBJECTDIR}/usart_sw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/usart_sw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/usart_sw.lst\\\" -e\\\"${OBJECTDIR}/usart_sw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/usart_sw.o\\\" \\\"usart_sw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/usart_sw.o"
	@${FIXDEPS} "${OBJECTDIR}/usart_sw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/rom.o: rom.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rom.o.d 
	@${RM} ${OBJECTDIR}/rom.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/rom.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/rom.lst\\\" -e\\\"${OBJECTDIR}/rom.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/rom.o\\\" \\\"rom.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/rom.o"
	@${FIXDEPS} "${OBJECTDIR}/rom.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
else
${OBJECTDIR}/delays.o: delays.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/delays.o.d 
	@${RM} ${OBJECTDIR}/delays.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/delays.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/delays.lst\\\" -e\\\"${OBJECTDIR}/delays.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/delays.o\\\" \\\"delays.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/delays.o"
	@${FIXDEPS} "${OBJECTDIR}/delays.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/main.o: main.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/main.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/main.lst\\\" -e\\\"${OBJECTDIR}/main.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/main.o\\\" \\\"main.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/main.o"
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/mcp23s17.o: mcp23s17.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/mcp23s17.o.d 
	@${RM} ${OBJECTDIR}/mcp23s17.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/mcp23s17.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/mcp23s17.lst\\\" -e\\\"${OBJECTDIR}/mcp23s17.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/mcp23s17.o\\\" \\\"mcp23s17.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/mcp23s17.o"
	@${FIXDEPS} "${OBJECTDIR}/mcp23s17.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/ssp_hw.o: ssp_hw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ssp_hw.o.d 
	@${RM} ${OBJECTDIR}/ssp_hw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/ssp_hw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/ssp_hw.lst\\\" -e\\\"${OBJECTDIR}/ssp_hw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/ssp_hw.o\\\" \\\"ssp_hw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/ssp_hw.o"
	@${FIXDEPS} "${OBJECTDIR}/ssp_hw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/databus.o: databus.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/databus.o.d 
	@${RM} ${OBJECTDIR}/databus.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/databus.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/databus.lst\\\" -e\\\"${OBJECTDIR}/databus.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/databus.o\\\" \\\"databus.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/databus.o"
	@${FIXDEPS} "${OBJECTDIR}/databus.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/addressbus.o: addressbus.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/addressbus.o.d 
	@${RM} ${OBJECTDIR}/addressbus.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/addressbus.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/addressbus.lst\\\" -e\\\"${OBJECTDIR}/addressbus.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/addressbus.o\\\" \\\"addressbus.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/addressbus.o"
	@${FIXDEPS} "${OBJECTDIR}/addressbus.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/usart_hw.o: usart_hw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart_hw.o.d 
	@${RM} ${OBJECTDIR}/usart_hw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/usart_hw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/usart_hw.lst\\\" -e\\\"${OBJECTDIR}/usart_hw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/usart_hw.o\\\" \\\"usart_hw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/usart_hw.o"
	@${FIXDEPS} "${OBJECTDIR}/usart_hw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/usart_sw.o: usart_sw.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart_sw.o.d 
	@${RM} ${OBJECTDIR}/usart_sw.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/usart_sw.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/usart_sw.lst\\\" -e\\\"${OBJECTDIR}/usart_sw.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/usart_sw.o\\\" \\\"usart_sw.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/usart_sw.o"
	@${FIXDEPS} "${OBJECTDIR}/usart_sw.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
${OBJECTDIR}/rom.o: rom.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rom.o.d 
	@${RM} ${OBJECTDIR}/rom.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/rom.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION) -u  -l\\\"${OBJECTDIR}/rom.lst\\\" -e\\\"${OBJECTDIR}/rom.err\\\" $(ASM_OPTIONS)    -o\\\"${OBJECTDIR}/rom.o\\\" \\\"rom.asm\\\" 
	@${DEP_GEN} -d "${OBJECTDIR}/rom.o"
	@${FIXDEPS} "${OBJECTDIR}/rom.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE)   -p$(MP_PROCESSOR_OPTION)  -w -x -u_DEBUG -z__ICD2RAM=1 -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"   -z__MPLAB_BUILD=1  -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -odist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
else
dist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE)   -p$(MP_PROCESSOR_OPTION)  -w  -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"   -z__MPLAB_BUILD=1  -odist/${CND_CONF}/${IMAGE_TYPE}/ASM452.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
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

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
