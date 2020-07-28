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
CONFIG OSC = HSPLL         ; Oscillator Selection bits (HS oscillator)
CONFIG OSCS = OFF       ; Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))

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
  
#define         SRAM_CS1_PIN           1
#define         SRAM_CS1_LAT           LATA
#define         SRAM_CS1_TRIS          TRISA
 
#define         SRAM_OE_PIN           2
#define         SRAM_OE_LAT           LATA
#define         SRAM_OE_TRIS          TRISA 
 
#define         SRAM_WE_PIN           3
#define         SRAM_WE_LAT           LATA
#define         SRAM_WE_TRIS          TRISA 

#define		Z80_RESET_PIN         4
#define		Z80_RESET_LAT         LATB
#define		Z80_RESET_TRIS        TRISB
 
#define		Z80_BUSACK_PIN        2
#define		Z80_BUSACK_LAT        LATB
#define		Z80_BUSACK_TRIS       TRISB
#define		Z80_BUSACK_PORT       PORTB 
 
#define		Z80_WAIT_PIN         1
#define		Z80_WAIT_LAT         LATB
#define		Z80_WAIT_TRIS        TRISB
 
SRAM_CS1_HI MACRO
	    bsf     SRAM_CS1_LAT, SRAM_CS1_PIN
	    ENDM 
 
SRAM_CS1_LO MACRO
	    bcf     SRAM_CS1_LAT, SRAM_CS1_PIN
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
	    
EXTERN usart_init
EXTERN usart_putchar
EXTERN usart_getchar
EXTERN usart_newline
EXTERN usart_getmessages
EXTERN usart_hex2ascii
	    
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
		
		;variables used in interrupts
		int_temp
		int_temp2
		int_temp3
		
		temp
		sram_temp
		
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
		movff	STATUS,STATUS_TEMP	;save STATUS register
		movff	WREG,WREG_TEMP		;save working register
		movff	BSR,BSR_TEMP		;save BSR register
		;pull wait line low
		bcf Z80_WAIT_LAT, Z80_WAIT_PIN
		
		;movff LATD, WREG
		
		; should check M1 line low as well
		;	*** high priority interrupt code goes here ***
		;Check ext INT0
		btfsc PIR1, PSPIF
		bra psp_handle
		bra int_exit
		
psp_handle:
		call addressbus_read
		movff PORTD, temp
		;xorlw 0x55
		;bnz psp_exit
		bsf LATB, 3
		;movff LATD, WREG
		;andlw 0x0f
		;movlw 'P'
		movf temp, w
		call usart_hex2ascii
		call usart_newline
		
		movf addressbus_val, w
		call usart_hex2ascii
		call usart_newline
		
		
		
psp_exit:    
		bcf PIR1, PSPIF
		bsf Z80_WAIT_LAT, Z80_WAIT_PIN
int_exit:
    
		movff	BSR_TEMP,BSR		;restore BSR register
		movff	WREG_TEMP,WREG		;restore working register
		movff	STATUS_TEMP,STATUS	;restore STATUS register
		retfie	FAST

;******************************************************************************
;Start of main program
; The main program code is placed here.

Main:
    call mcu_init
    
    bcf Z80_RESET_LAT, Z80_RESET_PIN
    bcf Z80_RESET_TRIS, Z80_RESET_PIN
    
    bsf Z80_BUSACK_TRIS, Z80_BUSACK_PIN
    
    bsf Z80_WAIT_LAT, Z80_WAIT_PIN
    bcf Z80_WAIT_TRIS, Z80_WAIT_PIN
    
    call usart_init
    
    call ssp_init
    
    ;bcf TRISA, RA5  ; SS pin
    
    ;call usart_init
    
    bsf LATA, 5
    bcf TRISA, 5
    
    bsf LATA, 0
    bcf TRISA, 0
    
    ;io req pins
    ;int
    ;bcf TRISD, 6
    ;bcf LATD, 6
    ;wr
    ;bsf TRISD, 5
    bcf LATB, 3
    bcf TRISB, 3
    
    call databus_init
    call addressbus_init
    
    ;call sram_deselect
    call sram_init
    
;    SRAM_WRITE 0x0000, 0x00
;    SRAM_WRITE 0x0001, 0x00
;    SRAM_WRITE 0x0002, 0x00
;    SRAM_WRITE 0x0003, 0xc3
;    SRAM_WRITE 0x0004, 0x00
;    SRAM_WRITE 0x0005, 0x00
;    SRAM_WRITE 0x0006, 0x76
;    SRAM_WRITE 0x0007, 0x00
    
    SRAM_WRITE 0x0000, 0x3e
    SRAM_WRITE 0x0001, 0x87
    SRAM_WRITE 0x0002, 0xd3
    SRAM_WRITE 0x0003, 0x07
    SRAM_WRITE 0x0004, 0xc3
    SRAM_WRITE 0x0005, 0x00
    SRAM_WRITE 0x0006, 0x00
    SRAM_WRITE 0x0007, 0x00
    SRAM_WRITE 0x0008, 0x00
    SRAM_WRITE 0x0009, 0x00
    
    ;nop
    ;nop
    
;    SRAM_WRITE 0x0000, 0x01
;    SRAM_WRITE 0x0001, 0x3e
;;    SRAM_WRITE 0x0002, 0x05
;;    SRAM_WRITE 0x0003, 0xd3
;    SRAM_WRITE 0x0002, 0x00
;    SRAM_WRITE 0x0003, 0x00
;    SRAM_WRITE 0x0004, 0xc3
;    SRAM_WRITE 0x0005, 0x00
;    SRAM_WRITE 0x0006, 0x76
    
    
    call release_control
 
    call z80_reset
    call become_iomode
    ;call z80_reset
    
    
    ;call z80_reset
main_loop:
    
    ;xorlw 0x80
;    btfss temp, 7
;    bra main_loop
    
    bsf LATA, 0
    ;bsf LATA, 1
    ;SRAM_CS2_HI
    
    ;movlw 0x0f
    ;call usart_hex2ascii
    ;call usart_putchar
    
;    movlw 0x00
;    call addressbusmode_set
    
    movlw .255
    call delay_millis
;    movlw .255
;    call delay_millis
    
    ;---------------------------------------------------------------------------
;    SRAM_READ 0x0002
;    ;movwf temp
;    xorlw 0xd3
;    btfss STATUS, Z
;    goto main_loop
    
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
    SRAM_CS1_LO
    SRAM_OE_LO
    
    nop
    call databus_read
    movwf sram_temp
    
    SRAM_OE_HI
    SRAM_CS1_HI
    
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
    SRAM_CS1_LO
    SRAM_WE_LO
    
    SRAM_WE_HI
    SRAM_CS1_HI
    
    clrf addressbus_val
    clrf addressbus_val+1
    call addressbus_write
    
    ;movlw 0
    ;call databus_write
    
    return
;----------------------------------------------------------------
sram_deselect:
    SRAM_OE_HI
    SRAM_CS1_HI
    SRAM_WE_HI
    ;SRAM_CS2_LO
    
    return
;----------------------------------------------------------------
sram_init:
    ;sram pins
    
    bcf LATA, 1
    bcf TRISA, 1
    
    bcf LATA, 2
    bcf TRISA, 2
    
    bcf LATA, 3
    bcf TRISA, 3
    
    call sram_deselect
    
    return
;----------------------------------------------------------------
release_control:
    ; sram control lines
    bsf TRISA, 1
    bsf TRISA, 2
    bsf TRISA, 3
    
    movlw 0x01
    call addressbusmode_set
    
    movlw 0x01
    call databusmode_set
    return
;----------------------------------------------------------------
z80_reset:
    bcf Z80_RESET_LAT, Z80_RESET_PIN
    bcf Z80_RESET_TRIS, Z80_RESET_PIN
    
    movlw .50
    call delay_millis
    ;movlw .255
    ;call delay_millis
    ;movlw .255
    ;call delay_millis
    ;movlw .255
    ;call delay_millis
    
    bsf Z80_RESET_LAT, Z80_RESET_PIN
    ;bsf Z80_RESET_TRIS, Z80_RESET_PIN
    
wait_busack:
    btfss Z80_BUSACK_PORT, Z80_BUSACK_PIN
    bra  wait_busack    
    
    ;nop
    
    return
    
become_iomode:
    ; enable ioreq interrupts, falling edge
;    bcf INTCON2, INTEDG0
;    bcf INTCON, INT0IF
;    bsf INTCON, INT0IE
    
    ;disable priority interrupts
    bcf RCON, IPEN
    
    ;PSP pins
    bsf TRISE, 0
    bsf TRISE, 1
    bsf TRISE, 2
    
    ;PSP mode
    bsf TRISE, 4
    
    setf TRISD
    
    ;PSP interrupt enable
    bcf PIR1, PSPIF
    bsf PIE1, PSPIE
    bsf INTCON, 6 ; PEIE
    bsf INTCON, 7 ; GIE
    
    return
    
    END


