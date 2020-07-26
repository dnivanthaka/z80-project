	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
	#include "mcp23s17.inc"
	
EXTERN ssp_init
EXTERN ssp_read
EXTERN ssp_write
	
UDATA
	
	MCP23X17_REG  RES 1
	MCP23X17_DATA RES 1
 
CODE

mcp23s17_init:
    MCP23S17_SS_ENA
    
    movlw MCP23S17_CMDWRITE
    call ssp_write
    
    movlw 0x00                        ; register
    call ssp_write
    
    movlw 0x00                        ; value
    call ssp_write
    
    MCP23S17_SS_DIS
    
    return
;-------------------------------------------------------------    
mcp23s17_write:
    MCP23S17_SS_ENA
    
    movlw MCP23S17_CMDWRITE
    call ssp_write
    
    movf MCP23X17_REG, w               ; register
    call ssp_write
    
    movf MCP23X17_DATA, w               ; value
    call ssp_write
    
    MCP23S17_SS_DIS
    
    return
;-------------------------------------------------------------    
mcp23s17_read:
    MCP23S17_SS_ENA
    
    movlw MCP23S17_CMDREAD
    call ssp_write
    
    movf MCP23X17_REG, w               ; register
    call ssp_write
    
    movf 0, w               ; value
    call ssp_write
    
    ;call ssp_read
    
    MCP23S17_SS_DIS
    
    return
;--------------------------------------------------------------
   
GLOBAL mcp23s17_init
GLOBAL mcp23s17_write
GLOBAL mcp23s17_read
    
GLOBAL MCP23X17_REG
GLOBAL MCP23X17_DATA

END
