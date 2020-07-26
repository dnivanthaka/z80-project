;******************************************************************************
;                                                                             *
;    Filename:                                                                *
;    Date:                                                                    *
;    File Version:                                                            *
;                                                                             *
;    Author:                                                                  *
;    Company:                                                                 *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files Required: P18F4550.INC                                             *
;                                                                             *
;******************************************************************************

	LIST P=18F452		;directive to define processor
	#include <p18f452.inc>	;processor specific variable definitions
	#include "mcp23s17.inc"
	

;******************************************************************************
;Configuration bits
;Microchip has changed the format for defining the configuration bits, please 
;see the .inc file for futher details on notation.  Below are a few examples.



; CONFIG1L
CONFIG OSC = HS         ; Oscillator Selection bits (HS oscillator)
CONFIG OSCS = ON       ; Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))

; CONFIG2L
CONFIG PWRT = OFF       ; Power-up Timer Enable bit (PWRT disabled)
CONFIG BOR = ON         ; Brown-out Reset Enable bit (Brown-out Reset enabled)
CONFIG BORV = 20        ; Brown-out Reset Voltage bits (VBOR set to 2.0V)

; CONFIG2H
CONFIG WDT = OFF        ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
CONFIG WDTPS = 128      ; Watchdog Timer Postscale Select bits (1:128)

; CONFIG3H
CONFIG CCP2MUX = ON     ; CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)

; CONFIG4L
CONFIG STVR = ON        ; Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
CONFIG LVP = OFF        ; Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

; CONFIG5L
CONFIG CP0 = OFF        ; Code Protection bit (Block 0 (000200-001FFFh) not code protected)
CONFIG CP1 = OFF        ; Code Protection bit (Block 1 (002000-003FFFh) not code protected)
CONFIG CP2 = OFF        ; Code Protection bit (Block 2 (004000-005FFFh) not code protected)
CONFIG CP3 = OFF        ; Code Protection bit (Block 3 (006000-007FFFh) not code protected)

; CONFIG5H
CONFIG CPB = OFF        ; Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code protected)
CONFIG CPD = OFF        ; Data EEPROM Code Protection bit (Data EEPROM not code protected)

; CONFIG6L
CONFIG WRT0 = OFF       ; Write Protection bit (Block 0 (000200-001FFFh) not write protected)
CONFIG WRT1 = OFF       ; Write Protection bit (Block 1 (002000-003FFFh) not write protected)
CONFIG WRT2 = OFF       ; Write Protection bit (Block 2 (004000-005FFFh) not write protected)
CONFIG WRT3 = OFF       ; Write Protection bit (Block 3 (006000-007FFFh) not write protected)

; CONFIG6H
CONFIG WRTC = OFF       ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write protected)
CONFIG WRTB = OFF       ; Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write protected)
CONFIG WRTD = OFF       ; Data EEPROM Write Protection bit (Data EEPROM not write protected)

; CONFIG7L
CONFIG EBTR0 = OFF      ; Table Read Protection bit (Block 0 (000200-001FFFh) not protected from Table Reads executed in other blocks)
CONFIG EBTR1 = OFF      ; Table Read Protection bit (Block 1 (002000-003FFFh) not protected from Table Reads executed in other blocks)
CONFIG EBTR2 = OFF      ; Table Read Protection bit (Block 2 (004000-005FFFh) not protected from Table Reads executed in other blocks)
CONFIG EBTR3 = OFF      ; Table Read Protection bit (Block 3 (006000-007FFFh) not protected from Table Reads executed in other blocks)
	
;CONFIG = _BOR_OFF_2L & _CPD_OFF_5H & _FOSC_HSPLL_HS_1H & _WDT_OFF_2H & _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L & _LVP_OFF_4L & _FCMEN_OFF_1H
;CONFIG = PLLDIV_10_1L ; For 40Mhz input = 40 / 10 = 4
;CONFIG = _CPUDIV_OSC1_PLL2_1L ; 96Mhz / 2
 ;   CONFIG	FOSC = XT_XT         ;XT oscillator, XT used by USB
 
#define		SERIAL_2_PORT		PORTA	
#define		SERIAL_2_TRIS		TRISA
#define         SERIAL_2_PIN           3
 
#define         SRAM_CS2_PIN           1
#define         SRAM_CS2_LAT           LATA
#define         SRAM_CS2_TRIS          TRISA
 
#define         SRAM_OE_PIN           2
#define         SRAM_OE_LAT           LATA
#define         SRAM_OE_TRIS          TRISA 
 
#define         SRAM_WE_PIN           3
#define         SRAM_WE_LAT           LATA
#define         SRAM_WE_TRIS          TRISA 
 
 
SRAM_CS2_HI MACRO
	    bsf     SRAM_CS2_LAT, SRAM_CS2_PIN
	    ENDM 
 
SRAM_CS2_LO MACRO
	    bcf     SRAM_CS2_LAT, SRAM_CS2_PIN
	    ENDM 

SRAM_OE_HI MACRO
	    bsf     SRAM_OE_LAT, SRAM_OE_PIN
	    ENDM 
	    
SRAM_OE_LO MACRO
	    bcf     SRAM_OE_LAT, SRAM_OE_PIN
	    ENDM 
	    
SRAM_WE_HI MACRO
	    bsf     SRAM_WE_LAT, SRAM_WE_PIN
	    ENDM 
	    
SRAM_WE_LO MACRO
	    bcf     SRAM_WE_LAT, SRAM_WE_PIN
	    ENDM 
	    
SERIAL_2_HI MACRO
	    bsf     SERIAL_2_PORT, SERIAL_2_PIN
	    ENDM
		    
SERIAL_2_LO MACRO
	    bcf     SERIAL_2_PORT, SERIAL_2_PIN
	    ENDM 
 
EXTERN ssp_init
EXTERN ssp_read
EXTERN ssp_write
 
EXTERN delay_us
EXTERN delay_millis

EXTERN mcp23s17_init 
EXTERN mcp23s17_write
EXTERN mcp23s17_read
EXTERN MCP23X17_REG, MCP23X17_DATA
	    
EXTERN databus_init
EXTERN databusmode_set
EXTERN databus_write
EXTERN databus_read

EXTERN addressbus_val	    
EXTERN addressbus_init
EXTERN addressbusmode_set
EXTERN addressbus_read
EXTERN addressbus_write
	    
SRAM_WRITE MACRO ADDR, VAL
	    movlw ADDR & 0xff
	    movwf addressbus_val
	    movlw (ADDR >> 8) & 0xff
	    movwf addressbus_val+1

	    movlw VAL
	    call sram_write
	   ENDM
	   
SRAM_READ MACRO ADDR
	    movlw ADDR & 0xff
	    movwf addressbus_val
	    movlw (ADDR >> 8) & 0xff
	    movwf addressbus_val+1

	    call sram_read
	   ENDM
	    
;******************************************************************************
;Variable definitions
; These variables are only needed if low priority interrupts are used. 
; More variables may be needed to store other special function registers used
; in the interrupt routines.

		CBLOCK	;0x080
		WREG_TEMP	;variable used for context saving 
		STATUS_TEMP	;variable used for context saving
		BSR_TEMP	;variable used for context saving
		
		temp
		sram_temp
		tdata
		counter
		ENDC

		CBLOCK	0x000
		;EXAMPLE		;example of a variable in access RAM
		ENDC
		

;******************************************************************************
;EEPROM data
; Data to be programmed into the Data EEPROM is defined here

;		ORG	0xf00000
;
;		DE	"Test Data",0,1,2,3,4,5

;******************************************************************************
;Reset vector
; This code will start executing when a reset occurs.

		ORG	0x0000

		goto	Main		;go to start of main code

;******************************************************************************
;High priority interrupt vector
; This code will start executing when a high priority interrupt occurs or
; when any interrupt occurs if interrupt priorities are not enabled.

		ORG	0x0008

		bra	HighInt		;go to high priority interrupt routine

;******************************************************************************
;Low priority interrupt vector and routine
; This code will start executing when a low priority interrupt occurs.
; This code can be removed if low priority interrupts are not used.

		ORG	0x0018

		movff	STATUS,STATUS_TEMP	;save STATUS register
		movff	WREG,WREG_TEMP		;save working register
		movff	BSR,BSR_TEMP		;save BSR register

;	*** low priority interrupt code goes here ***


		movff	BSR_TEMP,BSR		;restore BSR register
		movff	WREG_TEMP,WREG		;restore working register
		movff	STATUS_TEMP,STATUS	;restore STATUS register
		retfie

;******************************************************************************
;High priority interrupt routine
; The high priority interrupt code is placed here to avoid conflicting with
; the low priority interrupt vector.

HighInt:

;	*** high priority interrupt code goes here ***


		retfie	FAST

;******************************************************************************
;Start of main program
; The main program code is placed here.

Main:
    call mcu_init
    call usart_init
    
    call ssp_init
    
    ;bcf TRISA, RA5  ; SS pin
    
    ;call usart_init
    
    bsf LATA, 5
    bcf TRISA, 5
    
    bsf LATA, 0
    bcf TRISA, 0
    
    ;sram pins
    
    bcf LATA, 1
    bcf TRISA, 1
    
    bcf LATA, 2
    bcf TRISA, 2
    
    bcf LATA, 3
    bcf TRISA, 3
    
    ;call 
    
    call databus_init
    call addressbus_init
    
    call sram_deselect
    
    ;call mcp23s17_init
;    movlw 0x00
;    movwf MCP23X17_REG
;    
;    call mcp23s17_read
;    movwf temp
    
;    clrf addressbus_val
;    clrf addressbus_val+1
    
;    movlw 0x0f
;    movwf addressbus_val
;    movwf addressbus_val+1
    
;    movlw 0x00
;    movwf addressbus_val
    
;    movlw 0x55
;    call sram_write
    
    SRAM_WRITE 0x1fac, 0x5d
    
        
;    movlw IODIRA
;    movwf MCP23X17_REG
;    
;    movlw 0x00
;    movwf MCP23X17_DATA
;    
;    call mcp23s17_write
    
main_loop:
;    bcf STATUS, Z
;    movlw 0xff
;    subwf temp, w
;    btfss STATUS, Z
;    bra main_loop
    
;    movlw 0x14
;    movwf MCP23X17_REG
;    
;    movlw 0xff
;    movwf MCP23X17_DATA
;    
;    call mcp23s17_write
    
;    movlw GPIOA_0
;    movwf MCP23X17_REG
;    
;    movlw 0x80
;    movwf MCP23X17_DATA
;    
;    call mcp23s17_write
    ;movwf temp
    
    ;xorlw 0x80
;    btfss temp, 7
;    bra main_loop
    
    bsf LATA, 0
    ;bsf LATA, 1
    ;SRAM_CS2_HI
    
;    movlw 'X'
;    call usart_putchar
    
;    movlw 0x00
;    call addressbusmode_set
    
    movlw .255
    call delay_millis
;    movlw .255
;    call delay_millis
    
    ;---------------------------------------------------------------------------
    SRAM_READ 0x1fac
    ;movwf temp
    xorlw 0x5d
    ;btfss STATUS, Z
    bnz main_loop
;    movlw 0x14
;    movwf MCP23X17_REG
;    
;    movlw 0xff
;    movwf MCP23X17_DATA
;;    
;    call mcp23s17_write
    
    bcf LATA, 0
    ;bcf LATA, 1
    ;SRAM_CS2_LO
    
    ;movlw 'Y'
    ;call usart_putchar
    
    movlw .255
    call delay_millis
;    movlw .255
;    call delay_millis
    
    bra main_loop


;******************************************************************************
;End of program
    
mcu_init:
    clrf ADCON0
    movlw 0x0f
    movwf ADCON1                 ; disabling AD converter
    
    clrf CCP1CON
    clrf CCP2CON                ; disable comparator
    
    SERIAL_2_HI
    
    bcf     SERIAL_2_TRIS, SERIAL_2_PIN
    
    return
;-------------------------------------------------------------    
usart_init:
    bcf TRISC, TRISC6            ; tx pin
    ;bsf TRISC, TRISC7            ; rx pin
    
    clrf TXREG
    clrf RCREG
    ;bsf PIR1, TXIF
    
    movlw 0x40                   ; 19200
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
usart_hex2ascii:
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
;----------------------------------------------------------------------
;Software serial TX, 8N1 format
TXPoll_sw:                           ;1200bps
        movwf   tdata
	movlw   .8                 ;8N1
	movwf   counter
	
	SERIAL_2_LO                ;start bit
	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us
	
sw_serial_loop
	
	rrncf tdata, f
	btfss STATUS, C
	goto  sw_serial_zero
	goto  sw_serial_one
	
sw_serial_zero
	SERIAL_2_LO
	goto sw_serial_done
	
sw_serial_one
	SERIAL_2_HI

sw_serial_done	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us
	
	decfsz counter, f
	goto sw_serial_loop
	
	;stop bit
	SERIAL_2_HI
	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us

	return
;----------------------------------------------------------------    
sram_read:
    clrf sram_temp
    
    movlw 0x00
    call addressbusmode_set
    
    SRAM_WE_HI
    
    ; write address, should have the addressbus_val and addressbus_val+1 values set
    call addressbus_write
    
    movlw 0x01
    call databusmode_set
    
    SRAM_OE_HI
    SRAM_CS2_HI
    SRAM_OE_LO
    
    ;nop
    call databus_read
    movwf sram_temp
    
    SRAM_OE_HI
    SRAM_CS2_LO
    
;    movlw 0x00
;    call databusmode_set
    
    movf sram_temp, w
    
    return
;----------------------------------------------------------------    
sram_write:
    movwf sram_temp
    
    movlw 0x00
    call addressbusmode_set
    
    movlw 0x00
    call databusmode_set
    
    ; write address, should have the addressbus_val and addressbus_val+1 values set
    call addressbus_write
    
    movf sram_temp, w
    call databus_write
    
    SRAM_OE_HI
    SRAM_CS2_HI
    SRAM_WE_LO
    
    ;nop
    ;nop
    movlw .100
    call delay_millis
    
    SRAM_WE_HI
    SRAM_CS2_LO
    
    clrf addressbus_val
    clrf addressbus_val+1
    call addressbus_write
    
    ;movlw 0
    ;call databus_write
    
    return
;----------------------------------------------------------------
sram_deselect:
    SRAM_OE_HI
    SRAM_CS2_LO
    SRAM_WE_HI
    ;SRAM_CS2_LO
    
    return
;----------------------------------------------------------------
    END


