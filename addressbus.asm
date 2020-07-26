	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
#define IODIRA     0x00
#define IODIRB_0   0x01
#define IODIRB_1   0x10
#define IPOLA_0    0x02
#define IPOLA_1    0x01
#define GPINTENA_0 0x04
#define GPINTENA_1 0x02
#define GPINTENB_0 0x05
#define GPINTENB_1 0x12
#define DEFVALA_0  0x06
#define DEFVALA_1  0x03
#define DEFVALB_0  0x07
#define DEFVALB_1  0x13
#define INTCONA_0  0x08
#define INTCONA_1  0x04
#define INTCONB_0  0x09
#define INTCONB_1  0x14
#define IOCON_0    0x0A
#define IOCON_1    0x05
#define GPPUA_0    0x0C
#define GPPUA_1    0x06
#define GPPUB_0    0x0D
#define GPPUB_1    0x16
#define INTFA_0    0x0E
#define INTFA_1    0x07
#define INTFB_0    0x0F    
#define INTFB_1    0x17
#define INTCAPA_0  0x10
#define INTCAPA_1  0x08
#define INTCAPB_0  0x11
#define INTCAPB_1  0x18
#define GPIOA_0    0x12
#define GPIOA_1    0x09
#define GPIOB_0    0x13
#define GPIOB_1    0x19
#define OLATA_0    0x14
#define OLATA_1    0x0A
#define OLATB_0    0x15
#define OLATB_1    0x1A 	

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
GLOBAL addressbus_val
GLOBAL addressbus_init
GLOBAL addressbusmode_set
GLOBAL addressbus_read
GLOBAL addressbus_write    
    
	END


