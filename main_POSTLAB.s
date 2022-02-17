/*	
    Archivo:		main_preLAB.S
    Dispositivo:	PIC16F887
    Autor:		Javier Alejandro Pérez Marín 20183
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Contador de 4 bits con botón de aumento y de decremento
			mediante interrupciones y resistencias Pull up en PORTB.
			Contador de segundos de 4 bits en 2 7 segmentos
		        con interrupciones del TMR0.
    Hardware:		LEDs en el puerto A, 2 pb en puerto b y display 7 segmentos
			en PORTC y PORTD

    Creado:			16/02/22
    Última modificación:	16/02/22	
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

RESET_TMR0 MACRO
    BANKSEL TMR0	        ; Cambiamos al banco 1
    ;N=256-((20 ms)(4 MHz)/4*256) -> N= 217 aprox
    MOVLW   217                 ; Se mueve N al registro W
    MOVWF   TMR0	        ; Se le dan los 20 ms de delay a TMR0
    BCF	    T0IF	        ; Limpiamos la bandera de interrupción
    
    ENDM    
    
UP   EQU 0		      ; Definimos nombres a pines 0 y 1
DOWN EQU 1
     
; Variables
PSECT udata_bank0	      ; Common memory
    CONT_TMR:	DS 1	      ; 1 Byte
    V_PORTU:	DS 1	      ; 1 Byte
    V_PORTD:	DS 1	      ; 1 Byte
    
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
; --------------Interrupciones-------------
ORG 04h			     ; Posición 0004h para las interrupciones
PUSH:			     ; Se guarda el PC en la pila
    MOVWF  W_TEMP	     ; Movemos el registro W a la variable W_TEMP
    SWAPF  STATUS, W	     ; Se hace un swap de Nibbles del status y se guarda en W
    MOVWF  STATUS_TEMP	     ; Se pasa el registro W a la variable STATUS_TEMP
    
ISR:			     ; Rutina de interrupción
    BTFSC   T0IF	     ; Verficamos bandera de interrupción del TMR0
    CALL    CONT_TMR0	     ; Pasamos a subrutina de interrupción del TMR0
    
    BTFSC   RBIF	     ; Se verifica la bandera de cambio de estado de PORTB
    CALL    INT_IOCB	     ; Pasamos a subrutina INT_IOCB
POP:			     ; Se regresan las instrucciones de la pila al main
    SWAPF   STATUS_TEMP, W   ; Se hace swap de Nibbles de nuevo al STATUS
    MOVWF   STATUS	     ; Se mueve el registro W a STATUS
    SWAPF   W_TEMP, F	     ; Swap de Nibbles del registro W y se pasa a F
    SWAPF   W_TEMP, W	     ; Swap de Nibbles del registro W y se pasa a W
    
    RETFIE		     ; Se regresa de la interrupción 

;-------------Subrutinas de interrupciones------------
INT_IOCB:
    BANKSEL PORTB
    BTFSS   PORTB, UP	     ; Se verifica estado de los botones para inc o dec del contador
    INCF    PORTA	     ; Incremento del contador
    BTFSS   PORTB, DOWN
    DECF    PORTA	     ; Decremento del contador
    
    BCF	    RBIF	     ; Se limpia la flag de cambio de estado del PORTB		
    
    RETURN

CONT_TMR0:
    RESET_TMR0		     ; Reinicio TMR0
    INCF    CONT_TMR	     ; Se aumenta el valor de repeticiones de 20 ms
    MOVF    CONT_TMR, W	     ; Se mueve la cantidad de repeticiones a W
    SUBLW   50		     ; Se resta la literal con W (50 rep = 1000 ms)
    BTFSC   ZERO	     ; Se verifica si la última operación resultó en 0
    GOTO    INC_CONT2 	     ; Se pasa a la subrutina de aumento del CONT en DISPLAY
       
    RETURN

INC_CONT2:
    CLRF    CONT_TMR	     ; Limpiamos variable de repeticiones TMR0
    INCF    V_PORTU	     ; Se aumenta el valor del contador en C
    MOVF    V_PORTU, W       ; Valor de contador de unidades a W para buscarlo en la tabla
    CALL    TABLA	     ; Buscamos caracter de CONT de unidades en la tabla
    MOVWF   PORTC	     ; Guardamos caracter de CONT de unidades
    
    MOVF    V_PORTU, W       ; Valor de contador de unidades a W
    SUBLW   10		     ; Se verifica si ya llegamos al valor de 10 seg
    BTFSC   ZERO	     ; Se verifica si la última operación resultó en 0
    CALL    INC_CONTDEC	     ; Pasamos a subrutina de contador de decenas
    MOVF    V_PORTD, W	     ; Pasamos el valor del 7 segmentos de decenas
    CALL    TABLA	     ; Buscamos caracter de CONT de decenas en la tabla
    MOVWF   PORTD	     ; Guardamos caracter de CONT de decenas
    
    RETURN

INC_CONTDEC:
    CLRF    V_PORTU	     ; Reinicio contador de unidades
    INCF    V_PORTD	     ; Incremento contador de unidades
    MOVF    V_PORTU, W       ; Valor de contador de unidades a W para buscarlo en la tabla
    CALL    TABLA	     ; Buscamos caracter de CONT de unidades en la tabla
    MOVWF   PORTC	     ; Guardamos caracter de CONT de unidades
    
    MOVF    V_PORTD, W       ; Valor de contador de decenas a W
    SUBLW   6		     ; Se verifica que el contador de decenas no sea de 6
    BTFSC   ZERO	     ; Se verifica si la última operación resultó en 0
    CLRF    V_PORTD	     ; Reinicio de contador de decenas
    
    RETURN
    
; CONFIG uCS
PSECT code, delta=2, abs
ORG 100h                      ; posición para el código
TABLA:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 0		; Posicionamos el PC en dirección 01xxh
    ANDLW   0x0F		; no saltar más del tamaño de la tabla
    ADDWF   PCL			; Apuntamos el PC a PCLATH + PCL + W
    retlw 00111111B ;0
    retlw 00000110B ;1
    retlw 01011011B ;2
    retlw 01001111B ;3
    retlw 01100110B ;4
    retlw 01101101B ;5
    retlw 01111101B ;6
    retlw 00000111B ;7
    retlw 01111111B ;8
    retlw 01101111B ;9
    retlw 01110111B ;10 (A)
    retlw 01111100B ;11 (b)
    retlw 00111001B ;12 (C)
    retlw 01011110B ;13 (d)
    retlw 01111001B ;14 (E)
    retlw 01110001B ;15 (F)
    
 ; ---------------CONFIGURACIÓN--------------
 MAIN:
; Configuración Inputs y Outputs
    CALL    CONFIG_PINES
; Configuración deL Oscilador (4 MHz)
    CALL    CONFIG_RELOJ
; Configuración Timer0
    CALL    CONFIG_TIMER0
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
    BCF	    TRISA, 0          ; Ra0 a Ra3 como salida
    BCF	    TRISA, 1
    BCF	    TRISA, 2
    BCF	    TRISA, 3
    
    BANKSEL TRISB	      ; Cambiamos de banco
    BSF	    TRISB, UP	      ; Rb0 y Rb1 como inputs
    BSF	    TRISB, DOWN
    
    BANKSEL OPTION_REG	      ; Cambiamos de banco
    BCF	    OPTION_REG,	7     ; PORTB pull-up habilitadas (RBPU)
    BANKSEL WPUB
    BSF	    WPUB, UP	      ; Se habilita registro de Pull-up para Rb0 y Rb1
    BSF	    WPUB, DOWN
    
    BANKSEL TRISC
    CLRF    TRISC	      ; PORTC como salida
    
    BANKSEL TRISD
    CLRF    TRISD	      ; PORTD como salida
        
    BANKSEL PORTA             ; Cambiamos de banco
    CLRF    PORTA	      ; Limpieza de puertos para que inicien en 0
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    
    RETURN
    
CONFIG_RELOJ:
    BANKSEL OSCCON	      ; Cambiamos de banco
    BSF	    OSCCON, 0	      ; Seteamos para utilizar reloj interno (SCS=1)
    
    ;Se modifican los bits 4 al 6 de OSCCON al valor de 101b para frecuencia de 2 MHz (IRCF=101b)
    BSF	    OSCCON, 6
    BCF	    OSCCON, 5
    BSF	    OSCCON, 4
    
    RETURN
 
CONFIG_TIMER0:
    BANKSEL OPTION_REG	        ; Cambiamos de banco
    BCF	    T0CS	        ; Seteamos TMR0 como temporizador(T0CS)
    BCF	    PSA		        ; Se asigna el prescaler a TMR0(PSA)
   ; Se setea el prescaler a 256 BSF <2:0>
    BSF	    PS2		        ; PS2
    BSF	    PS1		        ; PS1
    BSF	    PS0		        ; PS0
    
    RESET_TMR0			; Macro
    
    RETURN
   
ENABLE_INTS:
    BANKSEL INTCON
    BSF	    GIE		      ; Se habilitan todas las interrupciones
    BSF	    RBIE	      ; Se habilita la interrupción de cambio de estado de PORTB	          
    BCF	    RBIF	      ; Flag de cambio de estado de PORTB
    BSF	    T0IE	      ; Se habilita interrupción del TMR0
    BCF	    T0IF	      ; Flag de interrupción TMR0
    
    RETURN
    
CONFIG_IOCRB:
    BANKSEL TRISA	      ; Cambio de banco
    BSF	    IOCB, UP	      ; Se habilita interrupción de cambio de estado para Rb0 y Rb1
    BSF	    IOCB, DOWN
    
    BANKSEL PORTA	      ; Cambio de banco
    MOVF    PORTB, W	      ; Al leer termina la condición de mismatch
    BCF	    RBIF	      ; Se limpia la flag de cambio de estado de PORTB
    
    RETURN
    

END






