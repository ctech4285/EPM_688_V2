include <p12F683.inc>
     __config (_INTRC_OSC_NOCLKOUT & _WDT_ON & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _IESO_OFF & _FCMEN_OFF)



;runs on PIC12F683
;Andreas Jochum


;GPIOs
;GPIO,0 Output, Data, high=on, low=off, output to pin 4 on JP2, ICSP
;GPIO,1 Output, Charge enable, ICSP pin 5 on JP2
;GPIO,2 Input Button, Output, LED
;GPIO,3 Input, PWM 
;GPIO,4 Output, Thyristor out for on
;GPIO,5 Output, Thyristor out for off



;Interrupt_Call
;Button  			->go to Button
;Timer_overflow    	->go to NO_PWM
;PWM    			->go to PWM
	







	 
	cblock 0x20				; Variables starting at this address
	Delay:2					; Define two registers for the Delay and Delay + 1
	Off_Pulse_Lengh			
	On_Pulse_Lengh
	Center_Pulse_Lengh
	GPIO_Last
	GPIO_Current			;needed to find out if we had a a trigger on rising or falling edge
	Status1					;100 =undefined 10=on 1=off
	PWM_Lengh	
	PWM_timout_counter_rollover


	PWM_Counter
	PWM_Last
	PWM_current
	
	 endc

     org 0                ; Start at flash address 0
Start:
	goto Setup
	nop
	nop
	nop
	goto Interrupt_Call

	
Setup:

	Banksel OSCCON
    movlw  	0x70			
	movwf  	OSCCON			; Set the internal clock speed to 8 Mhz 
	movlw  	0x07
	Banksel CMCON0

    movwf  	CMCON0			; Turn off Comparator (GP0, GP1, GP2)
	Banksel ANSEL
	clrf   	ANSEL			; Make all ports as digital I/O



	Banksel WDTCON
	BsF WDTCON,WDTPS0		;Watchdog Timer Period Select bits
	BcF WDTCON,WDTPS1		;1010= 1:32768
	BcF WDTCON,WDTPS2		;1024ms
	BsF WDTCON,WDTPS3
	BSF WDTCON,SWDTEN
	BSF WDTCON,WDTPS0		;Software Enable or Disable the Watchdog Timer (turn off for now)																	
	
	Banksel OPTION_REG
	BSF	OPTION_REG,7		;GPIO Pull-up Enable bit
	BSF	OPTION_REG,INTEDG	;Interrupt Edge Select bit
	BCF OPTION_REG,T0CS		;Timer0 Clock Source Select bit
	BCF OPTION_REG,T0SE		;Timer0 Source Edge Select bit (irelevant)
	BCF OPTION_REG,PSA		;Prescaler Assignment bit  (Assinged to Timer0)
	BSF	OPTION_REG,PS0		;Prescaler Rate Select bits
	BSF OPTION_REG,PS1		;111 = 1:256, 32ms 
	BSF	OPTION_REG,PS2		;

	Banksel WPU
	BCF 	WPU,2			;disable weak pull up on gpio 2 (button)
	

	Banksel TRISIO	
	BCF		TRISIO,0		;GPIO,0 output
	BCF		TRISIO,1		;GPIO,1 output
	BSF		TRISIO,2		;GPIO,2 input
	BSF		TRISIO,3		;GPIO,3 input
	BCF		TRISIO,4		;GPIO,4 output
	BCF		TRISIO,5		;GPIO,5 output




	Banksel INTCON
	BSF		INTCON,GIE		;Global Interrupt Enable bit
	BCF		INTCON,PEIE		;Peripheral Interrupt Enable bit
	BSF		INTCON,T0IE		;Timer0 Overflow Interrupt Enable bit
	BSF		INTCON,INTE		;GP2/INT External Interrupt Enable bit
	BSF		INTCON,GPIE		;GPIO Change Interrupt Enable bit

	Banksel IOC				;interupt on change
	BCF		IOC,0			
	BCF		IOC,1
	BCF		IOC,2			;input button, don't need to enable on change
	BSF		IOC,3			;input PWM
	BCF		IOC,4
	BCF		IOC,5
	
	;Turn watchdog on
	CLRWDT					;clear the WDT just in case
	Banksel WDTCON
	BSF WDTCON,WDTPS0		;Software Enable or Disable the Watchdog Timer (on)
	CLRWDT					;clear the WDT just in case

	;Turn thyristors charger off
	Banksel GPIO
	BCF		GPIO,1			;charge disable GPIO,0
	
	BCF 	GPIO,4			;thyristor output low
	BCF 	GPIO,5			;thyristor output low

	;Define PWM Pulse width
	Banksel Off_Pulse_Lengh
	movlw	.010		;0x0A=08, or 1.25ms
	movwf	Off_Pulse_Lengh
	
	Banksel On_Pulse_Lengh
	movlw	.014			;0x0E=12, or 1.75ms
	movwf	On_Pulse_Lengh

	Banksel Center_Pulse_Lengh
	movlw	.010				;0x0C=10, or 1.5ms
	movwf	Center_Pulse_Lengh

	;Define how many times a PWM is missed before blinking LED once
	movlw   .030
	movwf	PWM_timout_counter_rollover


	;Counters usded to see how many consecutive on or off pulses we had
	
	movlw 	.002		
	movwf	PWM_Counter	


	movlw	0x0
	movwf	PWM_Last

	movlw	0x0
	movwf	PWM_current		

	;Set current status 
	Banksel Status1
	movlw	B'00000100' 	;init status variable, 100 =undefined 010=on 001=off
	movwf	Status1
	
	;wait before turning charger on, make sure the thyristors are off
	movlw	.001					;Wait 1 ms delay, 
	call	DelayMs	
	;turn charger on
	Banksel GPIO
	BSF		GPIO,1			;charge enable is on GPIO,1
	BCF 	GPIO,0			;data low


	;if we get a reset togle data pin twice

;	BSF	GPIO,0
;	BCF GPIO,0
;	BSF	GPIO,0
;	BCF GPIO,0

	
	goto Main_Loop
	
Main_Loop:
;we should be here at least once per second so reset the WDT
CLRWDT

goto Main_Loop
	
	
Interrupt_Call:
	
		Banksel INTCON
	;	BCF INTCON,GIE					;disable all interups
		;save the GPIO, needed to detect rision or falling edge
		Banksel GPIO
		movf GPIO_Current,0
		Banksel GPIO_Last
		movwf GPIO_Last					;save current value as last
		Banksel GPIO
		movf GPIO,0						;copy GPIO into GPIO_current
		Banksel GPIO_Current
		movwf GPIO_Current
		
		;check where the interrupt came from
		Banksel INTCON

		BTFSC	INTCON,T0IF
		goto	Timer						;if TOIF bit is set interrupt came from the timer
											;Timer rolled over, call no pwm alert time: 32ms
		
		BTFSC INTCON,INTF	
		goto 	Button						;interrupt came from GPIO2
		
	
	
		BTFSC	INTCON,GPIF				;if the interrupt came from other GPIO 
		goto 	PWM						;asume its GPIO3 PWM
		
	
		;if we ended up here something whent wrong clear the interrupt
		goto Return_From_Interrupt

Timer
	
		DECFSZ PWM_timout_counter_rollover,1
			goto blah
		call PWM_warning					;Blink LED
blah
 		Banksel TMR0
		Clrf	TMR0					;reset the time0
		goto Return_From_Interrupt
	
Button
		Banksel INTCON
		BCF INTCON,GPIF
	

		BANKSEL INTCON
		BCF INTCON,INTE						;cleare gp2 external intrrupt flag
		Banksel Status1
		
											;Status is not off		
		BTFSC	Status1,1		
		goto Turn_Off						;Status is on, so Turn_Off
		
		
		;Status is not off turn on				
		goto Turn_On						
	
		goto Return_From_Interrupt
PWM	

		BTFSS	GPIO_Current,GP3	;
		goto Falling_Edge			;we got the falling edge
		BTFSC	GPIO_Current,GP3
		goto Rising_Edge			;we got a rising edge
		Banksel GPIO

Rising_Edge:
		Banksel TMR0
		Clrf	TMR0					;reset the time0
	
		goto Return_From_Interrupt
		


Falling_Edge:
		Banksel TMR0
		movf TMR0,0 		;move timer counter into w

		Banksel PWM_Lengh
		movwf PWM_Lengh		;
		


normo: 
		;test if PWM is greater then 0.75 and smaller then 2.25ms, 
		movlw .005 			; 0.75 ms		
		SUBWF PWM_Lengh,0
		Banksel STATUS
		BTFSS STATUS,C		;
		goto Invalid_PWM
		
		movlw .017 			; 2.25 ms, 	
		SUBWF PWM_Lengh,0
		Banksel STATUS
		BTFSC STATUS,C		;
		goto Invalid_PWM

		
		movf Off_Pulse_Lengh,0 			; 1.25 ms		
		SUBWF PWM_Lengh,0
		Banksel STATUS
		BTFSS STATUS,C		;
		goto Off_Pulse
		
		movf On_Pulse_Lengh,0			; 1.75ms	
		SUBWF PWM_Lengh,0
		Banksel STATUS
		BTFSC STATUS,C		;
		goto On_Pulse
		
		movlw 	.002		
		movwf	PWM_Counter
		goto Return_From_Interrupt


On_Pulse

		BTFSS PWM_Last,0
			goto Reset_PWM_Counter_On					;last was not on
		
		
		DECFSZ PWM_Counter
			goto Return_From_Interrupt							;hadnenough singals
		goto Turn_On

Off_Pulse
		BTFSC PWM_Last,0
			goto Reset_PWM_Counter_Off					;last was not on
		
		
		DECFSZ PWM_Counter
			goto Return_From_Interrupt								;had enough singals
		goto Turn_Off
		


Reset_PWM_Counter_On
		movlw 	.002		
		movwf	PWM_Counter
		movlw 	.001				;pwm last was on
		movwf	PWM_Last
		goto Return_From_Interrupt


Reset_PWM_Counter_Off
		movlw 	.002		
		movwf	PWM_Counter
		movlw 	.000				;pwm last was off
		movwf	PWM_Last
		goto Return_From_Interrupt
	



test	DECFSZ PWM_Lengh,1
			goto debug	
		goto Return_From_Interrupt
debug:	BSF		GPIO,0
		nop
		nop
		BCF		GPIO,0
		goto test



Invalid_PWM
		movlw 	.002		
		movwf	PWM_Counter
		goto Return_From_Interrupt

Turn_On
	movlw 	.002		
	movwf	PWM_Counter
	;disable capacitor charger, GPIO1
	Banksel GPIO
	bcf		GPIO,1
;	movlw	.050				; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine

	;set Status to On 010
	Banksel Status1
	BCF Status1,0
	BSF Status1,1
	BCF Status1,2

	;Pull data high
	Banksel GPIO
	BSF 	GPIO,0


	;set GPIO4 high for 1mS
	Banksel GPIO
	bsf		GPIO,5
	movlw	.005				; 1 ms delay
	call	DelayMs				; Call DelayMs Subroutine
	Banksel GPIO
	bcf		GPIO,5


	;wait till switch cycle is complete
;	movlw	.050				; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine

	
	;turn capacitor charger back on
	Banksel GPIO
	bsf		GPIO,1

	CLRWDT							;just in case
									;blink the LED 6 times, waiting ~600ms enough to charge the capacitor (charging takes longer at <5V inpu)
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay,
	CLRWDT							;clear the WDT just in case
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.200					;Wait 100 ms delay, 
	call	DelayMs	
	movlw	.200					;Wait 100 ms delay, 
	CLRWDT							;clear the WDT just in case
;2

	movlw 	.002		
	movwf	PWM_Counter
	;disable capacitor charger, GPIO1
	Banksel GPIO
	bcf		GPIO,1
;	movlw	.050				; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine

	;set Status to On 010
	Banksel Status1
	BCF Status1,0
	BSF Status1,1
	BCF Status1,2

	;Pull data high
	Banksel GPIO
	BSF 	GPIO,0


	;set GPIO4 high for 1mS
	Banksel GPIO
	bsf		GPIO,5
	movlw	.005				; 1 ms delay
	call	DelayMs				; Call DelayMs Subroutine
	Banksel GPIO
	bcf		GPIO,5


	;wait till switch cycle is complete
;	movlw	.050				; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine

	
	;turn capacitor charger back on
	Banksel GPIO
	bsf		GPIO,1
	CLRWDT							;just in case
									;blink the LED 6 times, waiting ~600ms enough to charge the capacitor (charging takes longer at <5V inpu)
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay,
	CLRWDT							;clear the WDT just in case
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED

	call	DelayMs	
	Banksel TMR0
	Clrf	TMR0					;reset the time0\

;3

		goto Return_From_Interrupt
		
Turn_Off
	movlw 	.002		
	movwf	PWM_Counter
	;disable capacitor charger, GPIO1
	Banksel GPIO
	bcf		GPIO,1
	movlw	.020				;; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine


	;set Status to Off 001
	Banksel Status1
	BSF Status1,0
	BCF Status1,1
	BCF Status1,2



	;Pull data Low
	Banksel GPIO
	BCF 	GPIO,0	;	debug

	;set GPIO4 high for 1ms
	Banksel GPIO
	bsf		GPIO,4
	movlw	.005				; 1 ms delay
	call	DelayMs				; Call DelayMs Subroutine
	Banksel GPIO
	bcf		GPIO,4


	;wait till switch cycle is complete
	movlw	.100				; needs testing if it can be reduced to 20ms
	call	DelayMs				; Call DelayMs Subroutine

	CLRWDT						;just in case
	;turn capacitor charger back on
	Banksel GPIO
	bsf		GPIO,1
	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	CLRWDT					;clear the WDT just in case

	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay,
	call	DelayMs	
	call Blink_LED
	movlw	.100					;Wait 100 ms delay, 
	call	DelayMs	
	call Blink_LED
	
	Banksel TMR0
	Clrf	TMR0					;reset the time0

	goto Return_From_Interrupt

	Return_From_Interrupt
	CLRWDT
	Banksel INTCON	
	BCF INTCON,INTF					;clear gp2 interrupt	
	BCF INTCON,T0IF					;reset time interrupt bit
	BCF INTCON,GPIF					;cleare gpio interrupt flag
	BSF INTCON,GIE					;enable all interups
	BCF	INTCON,T0IF				;clear timer interrupt
	BSF	INTCON,INTE	


	retfie							;return from interrupt	


	
PWM_warning
	movlw   .030
	movwf	PWM_timout_counter_rollover
	call Blink_LED
	return
	
	
	
	
	
		
	
	
Blink_LED:
						;Set GPIO 2 output
Banksel TRISIO
BCF		TRISIO,2		; GPIO,2 output
;lblink the led
Banksel GPIO
bsf		GPIO,2			; Set GP2 Output To High
movlw	.010					;Wait 10 ms delay, just in case there the PWM is noisy
call	DelayMs	

Banksel GPIO
bcf		GPIO,2			; Set GP2 Output To High
Banksel TRISIO
BSF		TRISIO,2		; GPIO,2 input
movlw	.010					;Wait 10 ms delay, just in case there the PWM is noisy
call	DelayMs
return					; return to the caller
	



	
	
	;----------------- DelayMs: Milisecond Delay Subroutine ----------------------
; Paramater: WREG = delay amount in milisecond, max: 255 milisecond
DelayMs:
     movwf  Delay + 1
DelayLoop:
     call   Delay1ms	
     decfsz Delay + 1,f   ; Decrease Delay + 1, If zero skip the next instruction
     goto   DelayLoop     ; Not zero goto DelayLoop
     return               ; return to the caller

;----------------- Delay1ms: 1 ms Delay Subroutine ---------------------------
Delay1ms:                 ; Total Delay: 1998 x 0.5us ~ 1 ms
     movlw  0x99
     movwf  Delay
DelayLoop1:
     decfsz Delay,f       ; Decrease Delay, If zero skip the next instruction
     goto   DelayLoop1
DelayLoop2:
     decfsz Delay,f       ; Decrease Delay1, If zero skip the next instruction
     goto   DelayLoop2    ; Not zero goto DelayLoop2
DelayLoop3:
     decfsz Delay,f       ; Decrease Delay1, If zero skip the next instruction
     goto   DelayLoop3    ; Not zero goto DelayLoop2
     return               ; Return to the caller

     end
