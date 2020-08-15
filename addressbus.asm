	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
	#include "mcp23s17.inc"

EXTERN mcp23s17_init 
EXTERN mcp23s17_write
EXTERN mcp23s17_read
EXTERN MCP23X17_REG, MCP23X17_DATA
	
	UDATA
	addressbus_val RES 2
 
	CODE

addressbus_init:
    movlw 0x01
    call addressbusmode_set
    
    return
;--------------------------------------------------------------------
addressbusmode_set:
    bcf STATUS, Z
    xorlw 0x01
    btfss STATUS, Z
    bra _ab_mode_output
    
_ab_mode_input:
    movlw IODIRA
    movwf MCP23X17_REG
    
    movlw 0xff
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    movlw IODIRB_0
    movwf MCP23X17_REG
    
    movlw 0xff
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    return
_ab_mode_output:    
    movlw IODIRA
    movwf MCP23X17_REG
    
    movlw 0x00
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    movlw IODIRB_0
    movwf MCP23X17_REG
    
    movlw 0x00
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    return

;----------------------------------------------------------------    
addressbus_read:
    clrf addressbus_val
    clrf addressbus_val+1
    
    ;low 8 bits
    movlw GPIOA_0
    movwf MCP23X17_REG
    
    call mcp23s17_read
    
    movwf addressbus_val
    
    ;high 8 bits
    movlw GPIOB_0
    movwf MCP23X17_REG
    
    call mcp23s17_read
    
    movwf addressbus_val+1
    
    return
;----------------------------------------------------------------    
addressbus_write:
    ;low 8 bits
    movlw GPIOA_0
    movwf MCP23X17_REG
    
    movf addressbus_val, w
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    ;high 8 bits
    movlw GPIOB_0
    movwf MCP23X17_REG
    
    movf addressbus_val+1, w
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    return
;---------------------------------------------------------------
addressbusmodeio_set:
    bcf STATUS, Z
    xorlw 0x01
    btfss STATUS, Z
    bra _ab_mode_output
    
_ab_modeio_input:
    movlw IODIRA
    movwf MCP23X17_REG
    
    movlw 0xff
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    movlw IODIRB_0
    movwf MCP23X17_REG
    
    movlw 0xff
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    return
_ab_modeio_output:    
    movlw IODIRA
    movwf MCP23X17_REG
    
    movlw 0x00
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    
    movlw IODIRB_0
    movwf MCP23X17_REG
    
    movlw 0xff
    movwf MCP23X17_DATA
    
    call mcp23s17_write
    return
;---------------------------------------------------------------    
GLOBAL addressbus_val
GLOBAL addressbus_init
GLOBAL addressbusmode_set
GLOBAL addressbus_read
GLOBAL addressbus_write
GLOBAL addressbusmodeio_set    
    
	END


