	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions	
	
    UDATA
	
	temp RES 1
	temp2 RES 1
 
    CODE
usart_init:
    bcf TRISC, TRISC6            ; tx pin
    bsf TRISC, TRISC7            ; rx pin
    
    clrf TXREG
    clrf RCREG
    ;bsf PIR1, TXIF
    
    movlw 0x6A
    ;movlw 0x40                   ; 19200
    ;movlw 0x1F                    ; 19200 @ 40Mhz
    movwf SPBRG
    
    movlw 0x24
    movwf TXSTA
    
    bcf TXSTA, SYNC
    bsf RCSTA, SPEN
    
    
    ;bsf PIE1, TXIE
    ;bsf PIE1, RCIE
    
    return
;--------------------------------------------------------------------    
usart_putchar:
    ;movwf UTXDATA
    ;INTCON, GIE
    
    btfss PIR1, TXIF
    bra usart_putchar
    
    movwf TXREG
    return
;--------------------------------------------------------------------    
usart_getchar:
    btfss PIR1, RCIF
    bra usart_getchar
    
    movf RCREG, w
    return
    
;--------------------------------------------------------------------
;usart_putstr:
;    movwf	FSR
;    
;outmessage:
;    movf FSR, w
;    incf FSR, f
;    call getmessages
;    xorlw 0
;    btfsc STATUS, Z
;    return
;    call TXPoll
;    goto outmessage
;    return
;--------------------------------------------------------------------    
usart_newline:
    movlw 0x0D
    call usart_putchar
    movlw 0x0A
    call usart_putchar
    return
;--------------------------------------------------------------------
usart_getmessages:
	addwf	PCL, f
	;dt	"Test Message", 0
	;dt	"Test Message2", 0
	dt "-------------------------------------------", 0x0D, 0x0A
	dt "Test Message", 0x0D, 0x0A
	dt "-------------------------------------------", 0
;--------------------------------------------------------------------
usart_hex2ascii_nb:
	movwf	temp
	btfss	temp, 3
	goto	ztn
	xorlw	0x08
	btfsc	STATUS, Z
	goto	ztn
	nop
	movf	temp, w
	xorlw	0x09
	btfsc	STATUS, Z
	goto	ztn
	nop
	movlw	0x07
	addwf	temp, f
ztn:
	movf	temp, w
	addlw	0x30
	call	usart_putchar
	return
;--------------------------------------------------------------------	
usart_hex2ascii:
    ;High byte first
    movwf temp2
    swapf temp2, w
    andlw 0x0f
    call usart_hex2ascii_nb
    movf temp2, w
    andlw 0x0f
    call usart_hex2ascii_nb
    
    return
	
;--------------------------------------------------------------------
GLOBAL usart_init
GLOBAL usart_putchar
GLOBAL usart_getchar
GLOBAL usart_newline
GLOBAL usart_getmessages
GLOBAL usart_hex2ascii	
	
    END

