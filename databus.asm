	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
	
	UDATA
	db_temp RES 1
 
	CODE

;----------------------------------------------------------------------
databus_init:
    ;Settingup pins as input
    movlw 0x01
    call databusmode_set
    
    return
    
;----------------------------------------------------------------------    
databusmode_set:
    bcf STATUS, Z
    xorlw 0x01
    btfss STATUS, Z
    bra _db_mode_output
    
_db_mode_input:
    bsf TRISC, RC0
    bsf TRISC, RC1
    bsf TRISC, RC2
    ;bsf TRISC, RC3
    bsf TRISD, RD0
    bsf TRISD, RD1
    bsf TRISD, RD2
    bsf TRISD, RD3
    bsf TRISD, RD4
    return
_db_mode_output:
    bcf TRISC, RC0
    bcf TRISC, RC1
    bcf TRISC, RC2
    ;bcf TRISC, RC3
    bcf TRISD, RD0
    bcf TRISD, RD1
    bcf TRISD, RD2
    bcf TRISD, RD3
    bcf TRISD, RD4
    
    return
;----------------------------------------------------------------------
databus_write:
    movwf db_temp
    
    bcf LATC, LATC0
    bcf LATC, LATC1
    bcf LATC, LATC2
    bcf LATD, LATD0
    bcf LATD, LATD1
    bcf LATD, LATD2
    bcf LATD, LATD3
    bcf LATD, LATD4
    
    btfsc db_temp, 0
    bsf LATC, LATC0
    btfsc db_temp, 1
    bsf LATC, LATC1
    btfsc db_temp, 2
    bsf LATC, LATC2
    btfsc db_temp, 3
    bsf LATD, LATD0
    btfsc db_temp, 4
    bsf LATD, LATD1
    btfsc db_temp, 5
    bsf LATD, LATD2
    btfsc db_temp, 6
    bsf LATD, LATD3
    btfsc db_temp, 7
    bsf LATD, LATD4
    
    return
;----------------------------------------------------------------------
databus_read:
    clrf db_temp
    
    btfsc PORTC, RC0
    bsf db_temp, 0
    btfsc PORTC, RC1
    bsf db_temp, 1
    btfsc PORTC, RC2
    bsf db_temp, 2
    btfsc PORTD, RD0
    bsf db_temp, 3
    btfsc PORTD, RD1
    bsf db_temp, 4
    btfsc PORTD, RD2
    bsf db_temp, 5
    btfsc PORTD, RD3
    bsf db_temp, 6
    btfsc PORTD, RD4
    bsf db_temp, 7
    
    movf db_temp, w
    
    return
;----------------------------------------------------------------------
    GLOBAL databus_init
    GLOBAL databusmode_set
    GLOBAL databus_write
    GLOBAL databus_read
    
	END


