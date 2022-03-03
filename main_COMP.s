/*	
    Archivo:		main_preLAB.S
    Dispositivo:	PIC16F887
    Autor:		Javier Alejandro Pérez Marín 20183
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Contador de segundos de 2 bits en hexadecimal 
		        con TMR0 cada 1.5 ms y TMR1 cada 500 ms, led
			intermitente con TMR2 cada 500 ms
    Hardware:		Display doble de 7 segmentos en PORTC, selector en
			PORTD y LED en RA0

    Creado:			1/03/22
    Última modificación:	2/03/22	
*/
    
PROCESSOR 16F887
// config statements should precede project file includes.
#include <xc.inc>
 
; CONFIG1
CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
CONFIG  PWRTE = OFF            ; Power-up Timer Enable bit (PWRT enabled)
CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)

CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

; Variables
PSECT udata_bank0	      ; Common memory
    CONT_LED:	 DS 1	      ; 1 Byte (contador de 50 ms con TMR2)
    CONT_MEDS:	 DS 1	      ; 1 Byte (Contador de 500 ms con TMR1)
    SEGS:	 DS 1	      ; 1 Byte (Contador de segundos)
    BANDERAS:	 DS 1	      ; 1 Byte (Banderas selector de display)
    UNIDADES:	 DS 1	      ; 1 Byte variable que almacena las unidades
    DECENAS:	 DS 1	      ; 1 Byte variable que almacena las decenas del contador
    DIV:	 DS 1	      ; 1 Byte variable que almacena valores de división
    DISPLAY:	 DS 2	      ; 2 Byte (Almacena en cada Byte la config de cada display)
    
RESET_TMR0 MACRO
    BANKSEL TMR0	        ; Cambiamos al banco 1
    MOVLW   250                 ; Se mueve N al registro W, N=256-((1.5 ms)(4 MHz)/4*256) -> N= 250 aprox
    MOVWF   TMR0	        ; Se le da delay a TMR0
    BCF	    T0IF	        ; Limpiamos la bandera de interrupción
    
    ENDM
    
RESET_TMR1 MACRO TMR1_H, TMR1_L	
    BANKSEL TMR1H
    MOVLW   TMR1_H	    ; Literal a guardar en TMR1H a W
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   TMR1_L	    ; Literal a guardar en TMR1L a W
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de interrupción de TMR1
    
    ENDM
    
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
    
;--------------Interrupciones-------------
ORG 04h			      ; Posición 0004h para las interrupciones
PUSH:			      ; Se guarda el w 
    MOVWF  W_TEMP	      ; Movemos el registro W a la variable W_TEMP
    SWAPF  STATUS, W	      ; Se hace un swap para no modificar el STATUS
    MOVWF  STATUS_TEMP	      ; Se pasa el registro STATUS a la variable STATUS_TEMP
    
ISR:			      ; Rutina de interrupción
    BTFSC   T0IF	      ; Verficamos bandera de interrupción del TMR0
    CALL    CONT_TMR0	      ; Pasamos a subrutina de interrupción del TMR0
    
    BTFSC   TMR1IF	      ; Verificamos bandera de interrupción del TMR1
    CALL    CONT_TMR1	      ; Pasamos a subrutina de interrupción del TMR1
    
    BTFSC   TMR2IF	      ; Verificamos bandera de interrupción del TMR2
    CALL    CONT_TMR2	      ; Pasamos a subrutina de interrupción del TMR2
                
POP:			      ; Se regresan los registros w y STATUS
    SWAPF   STATUS_TEMP, W   
    MOVWF   STATUS	     
    SWAPF   W_TEMP, F	     
    SWAPF   W_TEMP, W	     
    
    RETFIE		      ; Se regresa de la interrupción 
    
; CONFIG uCS
PSECT code, delta=2, abs
ORG 100h                      ; posición para el código
MAIN:
; Configuración Inputs y Outputs
    CALL    CONFIG_PINES
; Configuración deL Oscilador (4 MHz)
    CALL    CONFIG_RELOJ
; Configuración Timer0
    CALL    CONFIG_TIMER0
; Configuración Timer1
    CALL    CONFIG_TIMER1
; Configuración Timer2
    CALL    CONFIG_TIMER2  
; Configuración de interrupciones
    CALL    ENABLE_INTS

    BANKSEL PORTA
    CLRF	DECENAS
    CLRF	UNIDADES
    CLRF	SEGS
    CLRF	DIV
    
LOOP:
    CALL	CONFIG_DISPLAY	      ; Subrutina para traducción de valores a 7 seg   
    CALL	OBTENER_DECENAS	      ; Subrutina para obtener decenas de contador
    CALL	OBTENER_UNIDADES      ; Subrutina para obtener unidades de contador
        
    GOTO LOOP

CONFIG_PINES:
    BANKSEL ANSEL	      ; Cambiamos de banco
    CLRF    ANSEL	      ; I/O digital
    CLRF    ANSELH	    
        
    BANKSEL TRISA
    BCF	    TRISA, 0	      ; Ra0 como salida
    CLRF    TRISC	      ; PORTC como salida
    CLRF    TRISD	      ; PORTD como salida
        
    BANKSEL PORTA             ; Cambiamos de banco
    CLRF    PORTA
    CLRF    PORTC
    CLRF    PORTD
    
    RETURN
    
CONT_TMR0:
    RESET_TMR0               ; Reinicio TMR0
    CALL MOSTRAR_VAL
    
    RETURN
    
MOSTRAR_VAL:
    CLRF      PORTD          ; Limpiamos selector
    BTFSC     BANDERAS,0     ; Test bandera de display secundario
    GOTO      DISP_PRINC     ; Subrutina para display principal    

DISP_SEC:
    MOVF      DISPLAY, W     ; Movemos valor de disp principal a w
    MOVWF     PORTC	     ; Valor de la tabla a puerto
    BSF	      PORTD, 1	     ; Se enciende display secundario (derecho)
    
    BSF	      BANDERAS, 0    ; Se altera para pasar a modificar el principal en la siguiente repetición.
        
    RETURN

DISP_PRINC:
    MOVF    DISPLAY+1, W     ; Movemos valor de disp principal a w
    MOVWF   PORTC	     ; Valor de la tabla a puerto
    BSF	    PORTD, 0	     ; Se enciende display principal (Izquierdo)
    
    BCF	    BANDERAS, 0	     ; Se altera para pasar a modificar el secundario en la siguiente repetición.
        
    RETURN

    
OBTENER_DECENAS:
    CLRF	DECENAS
    CLRF	UNIDADES
    
    MOVF	SEGS,W	     ; Pasamos contador a variable de división
    MOVWF       DIV
    MOVLW	10	     ; Restamos 10 para obtener cantidad de decenas
    SUBWF	DIV, F	     
    INCF	DECENAS
    BTFSC	STATUS, 0    ; Se verifica si ocurrió BORROW (resultado aún mayor que 10)
    GOTO	$-4	     ; De ser así se continua restando
    DECF	DECENAS	     ; Se elimina la última añadición pues ya nos pasamos
    MOVLW	10	     ; Se regresa el valor a sus unidades antes de la resta 
    ADDWF	DIV, F

    RETURN  

OBTENER_UNIDADES:
    MOVLW	1		    ; Restamos 1 para obtener cantidad de unidades
    SUBWF	DIV, F	            
    INCF	UNIDADES
    BTFSC	STATUS, 0	    ; Se verifica si ocurrió BORROW (resultado aún mayor que 1)
    GOTO	$-4		    ; De ser así se continua restando
    DECF	UNIDADES
    MOVLW	1		    ; Se elimina la última añadición pues ya nos pasamos
    ADDWF	DIV, F		    ; Se regresa el valor a 0 en este caso

    RETURN
	
CONFIG_DISPLAY:               ; Subrutina para traducción de valores a 7 seg
    MOVF    UNIDADES, W	      ; Unidades
    CALL    TABLA	      ; Se pasa el valor a la tabla de 7 segmentos.
    MOVWF   DISPLAY           ; La configuración de pines va a la variable display_var
    
    MOVF    DECENAS, W        ; Decenas
    CALL    TABLA	      ; Se pasa el valor a la tabla de 7 segmentos.
    MOVWF   DISPLAY+1         ; La configuración de pines va al segundo byte de la variable display_var
    
    RETURN   
    
CONT_TMR1: ;Prelab
    RESET_TMR1 0x0B, 0xDC     ; Reiniciamos TMR1 para 500ms
    INCF    CONT_MEDS  	      ; Incremento contador de medio segundo
    BTFSS   CONT_MEDS, 1      ; Si cont_500=2 aumentamos en 1 segundo
    RETURN
    CLRF    CONT_MEDS
    INCF    SEGS	      ; Incrementamos segundos
    
    MOVF    SEGS, W	      ; Se limita el contador a 99 con una resta
    SUBLW   100
    BTFSC   ZERO	      ; Se verifica bandera de Zero para verificar valor
    CLRF    SEGS
    
    RETURN  
    
CONT_TMR2: ;Lab
    BCF	    TMR2IF	      ; Limpiamos bandera de interrupcion de TMR2
    INCF    CONT_LED	      ; Incrementamos contador led intermitente +50ms
    BTFSC   CONT_LED,3	      ; Verificamos si ya se contaron 10 veces 50ms (500 ms)
    BTFSS   CONT_LED,1
    RETURN
    CLRF    CONT_LED
    INCF    PORTA	      ; Incremento en PORTA (Led intermitente)
    
    RETURN
    
CONFIG_RELOJ:
    BANKSEL OSCCON	      ; Cambiamos de banco
    BSF	    OSCCON, 0	      ; Seteamos para utilizar reloj interno (SCS=1)
    
    ;Se modifican los bits 4 al 6 de OSCCON al valor de 110b para frecuencia de 4 MHz (IRCF)
    BSF	    OSCCON, 6
    BSF	    OSCCON, 5
    BCF	    OSCCON, 4
    
    RETURN
 
CONFIG_TIMER0:
    BANKSEL OPTION_REG	      ; Cambiamos de banco
    BCF	    T0CS	      ; Seteamos TMR0 como temporizador(T0CS)
    BCF	    PSA		      ; Se asigna el prescaler a TMR0(PSA)
   ; Se setea el prescaler a 256 BSF <2:0>
    BSF	    PS2		      ; PS2
    BSF	    PS1		      ; PS1
    BSF	    PS0		      ; PS0
    
    RESET_TMR0		      ; Macro
    
    RETURN
    
CONFIG_TIMER1:
    BANKSEL INTCON	      ; Cambiamos de banco
    BCF	    TMR1CS	      ; Cambiamos a reloj interno
    BCF	    T1OSCEN	      ; LP - OFF
    
    BSF	    T1CKPS1	      ; Prescaler 1:8
    BSF	    T1CKPS0
    
    BCF	    TMR1GE	      ; TMR1 contador
    BSF	    TMR1ON	      ; Encendemos TMR1
    
    RESET_TMR1 0x0B, 0xDC     ; TMR1 a 500 ms
    
    RETURN
    
CONFIG_TIMER2:
    BANKSEL PR2		      ; Cambiamos de banco
    MOVLW   240		      ; Valor para interrupciones cada 50 ms
    MOVWF   PR2		      ; Cargamos litaral a PR2
    
    BANKSEL T2CON	      ; Cambiamos de banco
    BSF	    T2CKPS1	      ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	      ;Postscaler 1:13
    BSF	    TOUTPS2
    BCF	    TOUTPS1
    BCF	    TOUTPS0
    
    BSF	    TMR2ON	      ; Encendemos TMR2
    
    RETURN
    
ENABLE_INTS:
    BANKSEL PIE1
    BSF	    TMR1IE	      ; Se habilita interrupción del TMR1
    
    BANKSEL INTCON
    BSF	    GIE		      ; Se habilitan todas las interrupciones
    BSF	    PEIE	      ; Se habilitan interrupciones de perifericos
    BSF	    T0IE	      ; Se habilita interrupción del TMR0
    BCF	    T0IF	      ; Flag de interrupción TMR0
    BCF	    TMR1IF	      ; Flag de interrupción TMR1
    
    RETURN
    
ORG 200h
TABLA:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
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
END

