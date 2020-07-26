	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
UDATA
	
	TXDATA RES 1
 
CODE	
;-------------------------------------------------------------    
ssp_init:
    bsf TRISC, TRISC4            ; SDI
    bcf TRISC, TRISC5            ; SDO
    bcf TRISC, TRISC3            ; SCK
    ;bcf TRISA, TRISA5            ; SS as input
    
    clrf SSPCON1
    clrf SSPBUF
    bcf PIR1, SSPIF
    
    ;movlw 0x3F
    ;andwf SSPSTAT, f
    
    ;movlw 0x40
    ;movlw 0
    ;movwf SSPSTAT
    bsf	SSPSTAT, CKE
    bcf	SSPSTAT, SMP
    
    bsf SSPCON1, CKP
    
    ;movlw 0x11
    ;movlw 0x20
    ;movwf SSPCON1
    
    ;movlw 0x20
    ;iorwf SSPCON1, f
    bcf SSPCON1, CKP
    bsf SSPCON1, SSPEN
    
    return
;-------------------------------------------------------------
ssp_write:
    movwf TXDATA
    ;bcf PIR1, SSPIF
    bcf SSPCON1, WCOL
    ;bcf SSPCON1, SSPOV
    
    ;movf SSPBUF, w
    ;movwf RXDATA
    
    ;bcf T2CON, TMR2ON
    ;clrf TMR2
    
    movf TXDATA, w
    movwf SSPBUF
    ;BSF T2CON, TMR2ON
    
    
write_complete:
    btfss PIR1, SSPIF
    bra write_complete
    
    bcf PIR1, SSPIF
    
    movf SSPBUF, w
    
    return
;------------------------------------------------------------
ssp_read:
    bcf PIR1, SSPIF
    ;bcf SSPCON1, SSPOV
    
    movf SSPBUF, w
    ;movwf RXDATA
    
    ;bcf T2CON, TMR2ON
    ;clrf TMR2
    
    movlw 0
    movwf SSPBUF
    ;BSF T2CON, TMR2ON
    
read_complete:
    btfss PIR1, SSPIF
    bra read_complete
    
    movf SSPBUF, w
    
    return
;------------------------------------------------------------
    
GLOBAL ssp_init
GLOBAL ssp_write
GLOBAL ssp_read
    
END