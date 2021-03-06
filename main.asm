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
	#include "config.inc"
	#include "mcp23s17.inc"
	
	

;******************************************************************************
;Configuration bits
;Microchip has changed the format for defining the configuration bits, please 
;see the .inc file for futher details on notation.  Below are a few examples.



; CONFIG1L

	
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
 
#define		BUS_LATCH_PIN         0
#define		BUS_LATCH_LAT         LATB
#define		BUS_LATCH_TRIS        TRISB
 
#define		ADDRESS_PIN           2
#define		ADDRESS_LAT           LATC
#define		ADDRESS_TRIS          TRISC
#define		ADDRESS_PORT          PORTC 
 
#define		SLAVE_RD_PIN          0
#define		SLAVE_RD_LAT          LATE
#define		SLAVE_RD_TRIS         TRISE
#define		SLAVE_RD_PORT         PORTE 
 
#define		SLAVE_WR_PIN          1
#define		SLAVE_WR_LAT          LATE
#define		SLAVE_WR_TRIS         TRISE
#define		SLAVE_WR_PORT         PORTE  

#define		SLAVE_CS_PIN          2
#define		SLAVE_CS_LAT          LATE
#define		SLAVE_CS_TRIS         TRISE
#define		SLAVE_CS_PORT         PORTE 
 
#define _EI_ bsf INTCON, 7 ; GIE
#define _DI_ bcf INTCON, 7 ; GIE
 
#define SLAVERX   0
#define SLAVETX   1
#define SLAVEBUSY 2 

#define ACK_BYTE 0x12
 
 
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
EXTERN addressbusmodeio_set	    
	    
EXTERN usart_init
EXTERN usart_putchar
EXTERN usart_getchar
EXTERN usart_newline
EXTERN usart_getmessages
EXTERN usart_hex2ascii
	    
EXTERN rom_data	    
	    
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
	   
	;org 0x6060 ; last 8k area

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
	
		slaveflags
		ENDC

		CBLOCK	0x000
		;EXAMPLE		;example of a variable in access RAM
		ENDC
		
		UDATA
		rx_count      RES 1
		tx_count      RES 1
		rx_buffer     RES 4
		tx_buffer     RES 4
		slave_cmd     RES 1
		
		

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
		
;		bcf Z80_WAIT_LAT, Z80_WAIT_PIN
;		bcf Z80_WAIT_TRIS, Z80_WAIT_PIN
		bcf ADDRESS_LAT, ADDRESS_PIN
		bcf ADDRESS_TRIS, ADDRESS_PIN
		;movlw 0x00
;		clrf addressbus_val
;		call addressbus_write
		
		
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

		
		;movff LATD, WREG
		
		; should check M1 line low as well
		;	*** high priority interrupt code goes here ***
		;Check ext INT0
		btfsc PIR1, PSPIF
		bra psp_handle
		bra int_exit
		
psp_handle:
		
		bsf LATB, 3
		;check if its a read or write
		btfss TRISE, IBF
		bra psp_read
		
		;psp_write, we got data, needs reading
		;check if the slave busy flag is set, if it is we ignore it
		btfsc slaveflags, SLAVEBUSY
		bra psp_exit
		
		; we ignore data bytes if we dont need it
		btfss slaveflags, SLAVERX
		bra psp_exit
		
		clrf FSR0H
		movlw rx_buffer
		movwf FSR0L
		
		;start count
		movf rx_count, w
		addwf FSR0L
		
		movf PORTD, w
		movwf INDF0
		
		incf rx_count, f
		
		;clear the IBOV flag
		bcf TRISE, IBOV
		
		;if the recv count is 2, we have a command and data byte
		;bcf slaveflags, SLAVEBUSY
		movf rx_count, w
		xorlw 0x02
		btfsc STATUS, Z
		bsf slaveflags, SLAVEBUSY
		;bsf slaveflags, SLAVERX
		
		bra psp_exit
psp_read:	
		;check if previously written byte is still present, if it is we just skip
		
;		btfsc TRISE, OBF
;		bra psp_exit
		
		btfsc TRISE, IBF
		bra psp_exit
		
		btfsc slaveflags, SLAVEBUSY
		bra psp_exit
		
		btfss slaveflags, SLAVETX
		bra psp_exit
		
		;if the read flag is set, we are waiting for more data
		;btfsc slaveflags, SLAVERX
		;bra psp_exit
		
		;check if we have data to transmit
		movf tx_count, w
		xorlw 0x00
		bz _psp_read_exit
		
		;we have data to transmit
		clrf FSR0H
		movlw tx_buffer
		movwf FSR0L
		
		decf tx_count, f
		movf tx_count, w
		addwf FSR0L
		
		movff INDF0, LATD
		
		bra psp_exit

_psp_read_exit:
		bcf slaveflags, SLAVETX
		bsf slaveflags, SLAVERX
		
		bra psp_exit
			
		
psp_exit:    
		; Clear IBOV flag
		bcf TRISE, IBOV
		bcf PIR1, PSPIF
int_exit:
    
		movff	BSR_TEMP,BSR		;restore BSR register
		movff	WREG_TEMP,WREG		;restore working register
		movff	STATUS_TEMP,STATUS	;restore STATUS register
		
		;btfsc slaveflags, SLAVEBUSY
		;bsf Z80_WAIT_TRIS, Z80_WAIT_PIN
		btfss slaveflags, SLAVEBUSY
		bsf ADDRESS_TRIS, ADDRESS_PIN
		
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
    
    bsf ADDRESS_TRIS, ADDRESS_PIN
    
    call usart_init
    
    call ssp_init
    
    ;bcf TRISA, RA5  ; SS pin
    
    ;call usart_init
    
    bsf LATA, 5
    bcf TRISA, 5
    
    bcf LATA, 0
    bcf TRISA, 0
    
    bcf LATB, 3
    bcf TRISB, 3
    
    call databus_init
    call addressbus_init
    
    ;call sram_deselect
    call sram_init
    
    call write_rom_to_sram
    
    call release_control
    call z80_reset
    call set_slave_mode
    
    ;enable receive
    bsf slaveflags, SLAVERX
    
main_loop:
    
    ;check for slave busy flag
;    movlw SLAVERX | SLAVEBUSY
;    xorwf slaveflags, w
;    btfsc STATUS, Z
;    call handle_slave_read
;    
    ;movlw SLAVETX | SLAVEBUSY
    ;xorwf slaveflags, w
    ;btfsc STATUS, Z
    ;call handle_slave_write
;    btfsc slaveflags, SLAVERX
    btfsc slaveflags, SLAVEBUSY
    call handle_slave_read
    
    
    ;call usart_getchar
    ;call usart_hex2ascii
    
;    btfsc slaveflags, SLAVETX
;    call handle_slave_write
    
;    btfsc slaveflags, SLAVERX
;    bra 
    
    ;btfsc slaveflags, SLAVETX
    ;call handle_slave_write
    
    ;read rx and tx flags
    
    ;xorlw 0x80
;    btfss temp, 7
;    bra main_loop
    
;    call addressbus_read
;    movf addressbus_val, w
;    call usart_hex2ascii
;    call usart_newline
    
    
;    bcf LATA, 0
    ;bsf LATA, 1
    ;SRAM_CS2_HI
    
    ;movlw 0x0f
    ;call usart_hex2ascii
    ;call usart_putchar
    
;    movlw 0x00
;    call addressbusmode_set
    
;    movlw .255
;    call delay_millis
;    movlw .255
;    call delay_millis
    
    ;---------------------------------------------------------------------------
;    SRAM_READ 0x0002
;    ;movwf temp
;    xorlw 0xd3
;    btfss STATUS, Z
;    goto main_loop
    
;    bcf LATA, 0
    ;bcf LATA, 1
    ;SRAM_CS2_LO
    
    ;movlw 'Y'
    ;call usart_putchar
    
;    movlw .255
;    call delay_millis
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
    
    bcf BUS_LATCH_LAT, BUS_LATCH_PIN ; A -> B, OE
    bcf BUS_LATCH_TRIS, BUS_LATCH_PIN
    
    ;Enabling TMR0
    movlw 0b11000111
    ; 1:256 prescale, low to high, 8bit, instruction cycle
    movwf T0CON
    
    return
;----------------------------------------------------------------    
handle_slave_read:
    bsf LATA, 0
    
    _DI_
    bcf slaveflags, SLAVEBUSY
    bcf slaveflags, SLAVERX
    clrf rx_count
    
    movf rx_buffer, w
    xorlw 0x02
    bz _usart_out
    
    movf rx_buffer, w
    xorlw 0x03
    bz _usart_in
    
    movf rx_buffer, w
    xorlw 0x04
    bz _set_tmr0
    
    movf rx_buffer, w
    xorlw 0x05
    bz _send_tmr0
    
    clrf rx_buffer
    clrf rx_buffer+1
    clrf rx_buffer+2
    clrf rx_buffer+3
    
    movlw .0
    movwf tx_count
    
    bra _send_done
    
_usart_out:
    
    clrf tx_count
    
    movf rx_buffer+1, w
    call usart_putchar
    
    bra _send_done
    
_usart_in:
    
    call usart_getchar
    movwf tx_buffer
    
    movlw .1
    movwf tx_count
    
    bra _send_done
    
_send_tmr0:
    movlw .1
    movwf tx_count
    
    movf TMR0, w
    movwf tx_buffer
    
    bra _send_done
    
_set_tmr0:
    movlw .0
    movwf tx_count
    
    movf rx_buffer+1, w
    movwf TMR0
    
    ;bra _send_done
    
; reply
_send_done:    

    movlw ACK_BYTE
    movwf LATD
    
    bsf slaveflags, SLAVETX
    
    _EI_
    
    ;bsf Z80_WAIT_TRIS, Z80_WAIT_PIN
    bsf ADDRESS_TRIS, ADDRESS_PIN
    
;    movlw 0x80
;    movwf addressbus_val
;    call addressbus_write
    
    return
;----------------------------------------------------------------
handle_slave_write:
    ;bsf slaveflags, SLAVEBUSY
    bcf slaveflags, SLAVETX
    clrf tx_count
    
    clrf tx_buffer
    clrf tx_buffer+1
    clrf tx_buffer+2
    clrf tx_buffer+3
    
    ;bcf slaveflags, SLAVETX
    
    bsf slaveflags, SLAVERX
    
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
    
    nop
    ;nop
    
    SRAM_WE_HI
    SRAM_CS1_HI
    
    ;clrf addressbus_val
    ;clrf addressbus_val+1
    ;call addressbus_write
    
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
    
    bcf SRAM_CS1_LAT, SRAM_CS1_PIN
    bcf SRAM_CS1_TRIS, SRAM_CS1_PIN
    
    bcf SRAM_OE_LAT, SRAM_OE_PIN
    bcf SRAM_OE_TRIS, SRAM_OE_PIN
    
    bcf SRAM_WE_LAT, SRAM_WE_PIN
    bcf SRAM_WE_TRIS, SRAM_WE_PIN
    
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
    
    call sram_deselect
    
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
    
    nop
    
    return
;-------------------------------------------------------------------    
z80_hold_in_reset:
    bcf Z80_RESET_LAT, Z80_RESET_PIN
    bcf Z80_RESET_TRIS, Z80_RESET_PIN
    
    return
;-------------------------------------------------------------------
z80_release_from_reset:
    bsf Z80_RESET_LAT, Z80_RESET_PIN
    
wait_busack2:
    btfss Z80_BUSACK_PORT, Z80_BUSACK_PIN
    bra  wait_busack2    
    
    nop
    return
;------------------------------------------------------------------- 
set_slave_mode:
    ; enable ioreq interrupts, falling edge
;    bcf INTCON2, INTEDG0
;    bcf INTCON, INT0IF
;    bsf INTCON, INT0IE
    
    ;disable priority interrupts
    bcf RCON, IPEN
    
    ;PSP pins
    bsf TRISE, 0 ; RD
    bsf TRISE, 1 ; RW
    bsf TRISE, 2 ; CS
    
    ;PSP mode
    bsf TRISE, 4
    
    setf TRISD
    
    ;PSP interrupt enable
    bcf PIR1, PSPIF
    bsf PIE1, PSPIE
    bsf INTCON, 6 ; PEIE
    bsf INTCON, 7 ; GIE
    
    bsf BUS_LATCH_LAT, BUS_LATCH_PIN ; Output disable
    
    bsf Z80_WAIT_TRIS, Z80_WAIT_PIN
    
;    ; set lower 8bit addressbus as output
;    movlw 0x00
;    call addressbusmodeio_set
    
    clrf tx_count
    clrf rx_count
    clrf slaveflags
    clrf tx_buffer
    clrf tx_buffer+1
    clrf tx_buffer+2
    clrf tx_buffer+3
    
    clrf rx_buffer
    clrf rx_buffer+1
    clrf rx_buffer+2
    clrf rx_buffer+3
    
    return
;-------------------------------------------------------
write_rom_to_sram:
    movlw upper rom_data
    movwf TBLPTRU
    movlw high rom_data
    movwf TBLPTRH
    movlw low rom_data
    movwf TBLPTRL
    
    ;flash memory read
    movlw 0x80
    movwf EECON1
    
    clrf addressbus_val
    clrf addressbus_val+1
    
_wr_sram_from_rom:
    TBLRD*+
    nop
    nop
    ;nop
    movf TABLAT, w
    call sram_write ;call to write sram, even
    incfsz addressbus_val, f
    bra _wr_sram_from_rom
    
    incf addressbus_val+1, f
    
    movf addressbus_val+1, w
    xorlw 0x04
    bnz _wr_sram_from_rom
    return
;--------------------------------------------------------------------------
write_to_rom:
    
    return
    END


