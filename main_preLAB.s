/*	
    Archivo:		main_preLAB.S
    Dispositivo:	PIC16F887
    Autor:		Javier Alejandro Pérez Marín 20183
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Contador de 4 bits con botón de aumento y de decremento
			mediante interrupciones y resistencias Pull up del PIC
    Hardware:		LEDs en el puerto A y 2 pb en puerto b

    Creado:			15/02/22
    Última modificación:	15/02/22	
*/
    
PROCESSOR 16F887
// config statements should precede project file includes.
#include <xc.inc>
 
; CONFIG1
CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)

CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

UP   EQU 0		      ; Definimos nombres a pines 0 y 1
DOWN EQU 1
 
; Status para interrupciones
PSECT udata_shr		      ; Common memory
   W_TEMP:	DS 1	      ; 1 Byte
   STATUS_TEMP: DS 1	      ; 1 Byte
    
; CONFIG Vector RESET    
PSECT resVect, class=CODE, abs, delta=2
ORG 00h                       ; posición 0000h para el reset


; ---------------vector reset--------------
resetVec:
    PAGESEL MAIN
    GOTO    MAIN

ORG 04h			     ; Posición 0004h para las interrupciones
PUSH:
    MOVWF  W_TEMP
    SWAPF  STATUS, W
    MOVWF  STATUS_TEMP
    
ISR:
   
POP:
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP, W
    RETFIE
    
; CONFIG uCS
PSECT code, delta=2, abs
ORG 100h                      ; posición para el código

 ; ---------------CONFIGURACIÓN--------------
 MAIN:
; Configuración Inputs y Outputs
    CALL    CONFIG_PINES
; Configuración deL Oscilador (1 MHz)
    CALL    CONFIG_RELOJ
; Configuración de interrupciones
    CALL    ENABLE_INTS
; Configuración de lectura de cambios en puerto B
    CALL    CONFIG_IOCRB

LOOP:
    GOTO LOOP

CONFIG_PINES:
    BANKSEL ANSEL	      ; Cambiamos de banco
    CLRF    ANSEL	      ; Ra como I/O digital
    CLRF    ANSELH	      ; Rb como I/O digital
    
    BANKSEL TRISA
    MOVLW   0h
    MOVWF   TRISA	      ; Ra0 a Ra3 como salida
    
    BANKSEL TRISB	      ; Cambiamos de banco
    BSF	    TRISB, UP	      ; Rb0 y Rb1 como inputs
    BSF	    TRISB, DOWN
    
    BANKSEL OPTION_REG	      ; Cambiamos de banco
    BCF	    OPTION_REG,	7     ; PORTB pull-up habilitadas
    BANKSEL WPUB
    BSF	    WPUB, UP	      ; Se habilita registro de Pull-up para Rb0 y Rb1
    BSF	    WPUB, DOWN	    
    
    BANKSEL PORTA             ; Cambiamos de banco
    CLRF    PORTA	      ; Limpieza de puertos para que inicie en 0
    CLRF    PORTB
    
    RETURN
    
CONFIG_RELOJ:
    BANKSEL OSCCON	      ; Cambiamos de banco
    BSF	    OSCCON, 0	      ; Seteamos para utilizar reloj interno (SCS=1)
    
    ;Se modifican los bits 4 al 6 de OSCCON al valor de 100b para frecuencia de 1 MHz (IRCF=100b)
    BSF	    OSCCON, 6
    BCF	    OSCCON, 5
    BCF	    OSCCON, 4
    
    RETURN
    
ENABLE_INTS:
    BSF	GIE		      ; INTCON
    BSF RBIE		          
    BSF	RBIF
    RETURN
    
CONFIG_IOCRB:
    BANKSEL TRISA
    BSF	    IOCB, UP
    BSF	    IOCB, DOWN
    
    BANKSEL PORTA
    MOVF    PORTB, W	    ; Al leer termina la condición de mismatch
    BCF	    RBIF
    RETURN
    
END
