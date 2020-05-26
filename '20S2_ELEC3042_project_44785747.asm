;
; JadonGohMajorProject.asm
;
; Created: 22/04/2020 11:50:49 AM
; Author : Jadon
;
;		44785747 Jadon Goh MQ


	;Constants

	.equ Alert_Sound = 300		;Frequency for alert sound
	.equ Evac_Sound_One = 100	;Frequency for evacuate sound 1
	.equ Evac_Sound_Two = 200	;Frequency for evacuate sound 2
	.equ ALERTSTATE_ON = 1		;States for controlling the alert tone change
	.equ ALERTSTATE_OFF = 2
	.equ EVAC_TONE_1 = 3		;States for controlling the evac tone change
	.equ EVAC_TONE_2 = 4
	.equ RESETSTATE_1 = 1		;States for the reset state machine
	.equ RESETSTATE_2 = 2
	.equ RESETSTATE_3 = 3
	.equ RESETSTATE_4 = 4
	.equ QUIESCENT_STATE = 1	;States for the sector and quiescent state machine
	.equ SECTORSTATE_1 = 2
	.equ SECTORSTATE_2 = 3
	.equ SECTORSTATE_3 = 4
	.equ ISOLATED_STATE = 1		;State for the isolated sector
	.equ ALERT_TONE = 1			;States for controlling which tone should be playing
	.equ EVAC_TONE = 2
	.equ ADC_RESET_TRIGGER = 1	;State for masking the reset button on the ADC conversion
	
	.dseg
buzzer: .byte 1					;Tone change
resetcounter:	.byte 1			;Reset Level
multipletriggers:	.byte 1		;Mutiple Sectors
sectorstate:	.byte 1			;Level counter		
isolatedtrigger:	.byte 1		;Isolated state trigger
tonecounter:	.byte 1			;Alert Evac Quiet
adcresettrigger:	.byte 1		; ADC conversion reset button state
	.cseg


	.org 0x00
	jmp RESETCODE ; Reset ; reset and interrupt vectors 
	jmp EmerIsoRes ; IRQ0 
	jmp Nothing ; IRQ1 
	jmp Nothing ; PCINT0 
	jmp Nothing ; PCINT1 
	jmp SectorTrigger ; PCINT2 
	jmp Nothing ; Watchdog Timeout 
	jmp ToneChange ; Timer2 CompareA 
	jmp Nothing ; Timer2 CompareB 
	jmp Nothing ; Timer2 Overflow 
	jmp Nothing ; Timer1 Capture 
	jmp Nothing ; Timer1 CompareA 
	jmp TwoSecondDelay ; Timer1 CompareB 
	jmp Nothing ; Timer1 Overflow
	jmp Nothing ; Timer0 CompareA 
	jmp Nothing ; Timer0 CompareB 
	jmp Nothing ; Timer0 Overflow 
	jmp Nothing ; SPI Transfer Complete
	jmp Nothing ; USART RX Complete
	jmp Nothing ; USART UDR Empty
	jmp Nothing ; USART TX Complete
	jmp ADCButton ; ADC Conversion Complete


RESETCODE:

	;Set Stack Pointers
	ldi r16,high(RAMEND) 
	out SPH,r16				; Set Stack Pointer to top of RAM
	ldi r16,low(RAMEND)
	out SPL,r16
	
	;Port B pin configure for the MCP
	ldi r16,0b00101110 ;SS, MOSI, SCK all outputs. MISO input
	out DDRB,r16
	sbi PORTB,2 ; and SS back high for as we dont want to send anymore data yet

	
	;Port C pin configure I2C/LCD
	ldi r16,0b00001110 ;Port C pins 0, 4 and 5 inputs with pullup resistors on 4 and 5. Pins 1,2,3 outputs
	out DDRC,r16
	ldi r16,0b00110000 
	out PORTC,r16

	;Port D pin configure (Interrupts and inputs)
	ldi r16,0b00000010 ;Port D 2,5,6 and 7 inputs with pull up resistor on for pin 1,2,5,6 and 7. Pin 1 output for serial communication.
	out DDRD,r16
	ldi r16,0b11100110 
	out PORTD,r16 

	;--------------------------------------------------------------Mega Interrupt Setup--------------------------------------------------------

; The interrupts im using are INT0, INT1 and PCINT23
	cbi EIMSK,INT0 ; mask INT0, INT1 and PCINT23 while we setup
	clr r16
	sts PCMSK2,r16
	ldi r16,(1<<ISC01) ; INT0 will trigger on falling edge
	sts EICRA,r16 
	ldi r16,(1<<PCIE2) ;Set PCIE2 for interrupt change on pin
	sts PCICR,r16
	sbi EIMSK,INT0 ; turn on INT0,INT1, PCINT23 ,PCINT22, PCINT21 as we're ready to receive interrupts
	ldi r16,(1<<PCINT23)|(1<<PCINT22)|(1<<PCINT21)
	sts PCMSK2,r16

;---------------------------------------------------------------------------ADCConversion Setup------------------------------------------------------------------------------

ADCConversion:
	ldi r16,(1<<REFS0)|(1<<ADLAR) ;Vcc Refernce Voltage, ADC0 Pin, Left Adjusted Result
	sts ADMUX,r16
	ldi r16,(1<<ADC0D) ;Digital Input Enabled Pin0
	sts DIDR0,r16
	ldi r16,(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADATE) ;Enable and start conversion, Free-running mode, ADC Conversion interrupt enabled, Division factor of 1024
	sts ADCSRA,r16

	

;------------------------------------------------------------------MCP23S17 Configuration-----------------------------------------------------------

;SPI setup
	ldi r16,(1<<SPE)|(1<<MSTR); setting master and enabling SPI operations
	out SPCR,r16 ; SCK is set fosc/4 => 4MHz
	clr r16 ; clear interrupt flags and oscillator mode.
	out SPSR,r16

;Configuring the MCP23S17

	ldi r20,0x00 ; address of register IODIRA (port A data direction)
	ldi r21,0b01110000 ; Pins 5,6,7 Inputs. 0,1,2 Outputs
	call SPISendCommandChip

	ldi r20,0x0c ; address of register GPPUA (port A GPIO Pullups)
	ldi r21,0b01110000 ; turn on pullups for sensors
	call SPISendCommandChip

	ldi r20,0x08 ; address of register INTCONA (port A IOC control register)
	ldi r21,0b00000000 ; Pin compared to previous value
	call SPISendCommandChip

	ldi r20,0x04 ; address of register GPINTENA (port A Interrupt on change)
	ldi r21,0b01010000 ; Interrupt on change for buttons
	call SPISendCommandChip

	;-------------------------------------------------------USART Setup----------------------------------------------------------------------------
	
	;Code provided by Rex
	clr	r16
	sts	UCSR0A,r16
	ldi	r16,0x18	; enable receiver and transmitter
	sts	UCSR0B,r16
	ldi	r16,0x06	; async, no parity, 1 stop, 8 bits
	sts	UCSR0C,r16
	clr	r16
	sts	UBRR0H,r16
	ldi	r16,0x67	; baud rate divisor 103 (16M/9600 - 1)
	sts	UBRR0L,r16
	call TextInitialise ;Create our 1st and 2nd line text bytes
;------------------------------------------------------------------Timers--------------------------------------------------------------------

	ldi r16,(1<<WGM21)	;Timer0 initialisation. Used for tone changes. Other timers are created in the calling functions.
	sts TCCR2A,r16		;CTC Mode
	ldi r16,255
	sts OCR2A,r16		;Compare A output 
	clr r16
	sts TCNT2,r16		;Begin timer at 0
	


;Constants
	clr r28	;Constant for tone change timers
	clr r27	;Isolated counter
	clr r26	;Isolated trigger
	clr r25 ;Sector trigger 
	clr r24	;Timer loop
	clr r23	;Timer value

;-----------------------------------------------------------------------------------------------------------------------------------------------------------------
;-----------------------------------------------------------Main Loop--------------------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------------------------------------------------------------------------------------

;Main loop of code is broken up into 4 sections. Quiescent, Level 1(Flash), Level 2(Flashing) and Level 3 (Solid)
;Single triggers will continue on its respective path
;Multiple triggers will jump to Level 3
;An Isolated button press will return to the Quiescent state
;An Emergency button press will return to the same position
;For the reset button. 1st press will return to the same spot, 2nd press will return to the Quiescent state, 3rd press will do a full reset

;Each level calls many different function and states that are either below or within the main loop.

;-----------------------------------------------------------Quiescent State--------------------------------------------------------------------------------------------

MainCode:
	call ResetMode ;Reset all counters, triggers, constants, timers, states and button masks
	call SleepMode	;Sleep until a command
MainCodeStart:
	ldi r16,QUIESCENT_STATE		;Set quiescent state
	sts sectorstate,r16
	call TriggeredText			;Display to serial moniter
MainCodeStartTwo:
	call AllSectorsOff			;Turn off all sectors
	clr r16
	sts isolatedtrigger,r16		;Clear the isolated trigger state
	ldi r23,90					;Half a second loop
MainCodeLoop:
	tst r25						
	brne LevelOne				;Branch to level 1 if any sector is triggered.
	call StartADCConversion		;Start ADCConversion
	call Timer0					;Call Timer0 which is a short delay
	call FullResetCheck			;Check if we are on the 3rd reset button press
	dec r23					
	brne MainCodeLoop			;Loop for half a second
	call IsolatedSectorFlash	;Flash any isolated sectors
	rjmp MainCodeStartTwo		;Repeat

FullResetCheck:	
	lds r16,resetcounter		;State check if we need to do a full reset
	cpi r16,RESETSTATE_4
	breq MainCode
	ret

ResetLevelCheck:
	lds r16,resetcounter
	cpi r16,RESETSTATE_3		;State check if we need to clear all triggered sectors and only have isolated sectors
	breq MainCodeStart
	ret

IsolatedSectorsCheck:
	lds r16,isolatedtrigger		;State check if we are now in an isolated state
	cpi r16,ISOLATED_STATE
	breq MainCodeStart
	ret

	;-----------------------------------------------------------Level One (Flash)----------------------------------------------------------------
;We enter this level if any sectors are triggered

LevelOne:
	ldi r16,SECTORSTATE_1	;Change the state to Level 1
	sts sectorstate,r16
	call TriggeredText		;Update the text
	call TriggeredSectorOn ;Turn any triggered sectors on	
	call Timer0				;Wait a brief moment
	call AllSectorsOff		;Turn them off
	ldi r24,2				;Start a timer loop
LevelOneLoopTwo:
	call IsolatedSectorFlash	;Flash any isolated sectors
	ldi r23,90					;Half a second
LevelOneLoopOne:	
	call Timer0					;Start the timer
	call StartADCConversion		;Start the ADC Conversion
	call SectorCheck			;Check which button needs to be held for 3 seconds
	call MultipleSectorsCheck	;Check if multiple buttons have been pressed
	call IsolatedSectorsCheck	;Check if the isolated button has been pressed
	call ResetLevelCheck		;Check if the reset button has been pressed
	dec r23
	brne LevelOneLoopOne		;Loop for half a second
	dec r24
	brne LevelOneLoopTwo		;Continue until the corresponding button has been held for 3 seconds
	rjmp LevelTwo		;Continue on to level 2









;-----------------------------------------------------------Level Two (Flashing)----------------------------------------------------------------

LevelTwo:
	ldi r16,SECTORSTATE_2	;Change the state to level 2
	sts sectorstate,r16
	ldi r24,6				;Set a timer loop
	ldi r27,1				;Set another timer loop, used for the isolated sector flash
	call ToneCheck			;Check which tone we are meant to play and turn on
	call TriggeredText		;Update the text
	call IsolatedSectorFlash	;Flash any isolated sectors
LevelTwoLoop:
	ldi r23,15				;1/12 of a second
	call TriggeredSectorOn	;Turn on any triggered sectors
LevelTwoOnLoop:
	call Timer0				;Same loop as level 1
	call StartADCConversion
	call SectorCheck
	call MultipleSectorsCheck
	call IsolatedSectorsCheck
	call ResetLevelCheck
	dec r23
	brne LevelTwoOnLoop
	ldi r23,15
	call AllSectorsOff
LevelTwoOffLoop:			;Same as the above loop except with any triggered sectors off 
	call Timer0
	call StartADCConversion
	call SectorCheck
	call MultipleSectorsCheck
	call IsolatedSectorsCheck
	call ResetLevelCheck
	dec r23
	brne LevelTwoOffLoop
	inc r27						
	sbrc r27,2					;Flash any isolated sectors
	call LevelTwoIsolatedFlash
	dec r24						
	brne LevelTwoLoop			;Keep looping until sector button his held
	rjmp LevelThree				;Go to level three

LevelTwoIsolatedFlash:			;Function for flashing the isolated sectors and resetting the timer loop
	call IsolatedSectorFlash
	ldi r27,1
	ret

	ToneCheck:					;Function to check which tone is to be played. 
	lds r16,tonecounter			;Change the tone counter to the evac tone state
	cpi r16,EVAC_TONE
	breq RemainInEvac
	call AlertTone				;Play altert tone
	ret
RemainInEvac:
	call EvacuateTone			;Play evac tone
	ret	

;-----------------------------------------------------------Level Three (Solid LED)----------------------------------------------------------------

LevelThree:
	ldi r16,SECTORSTATE_3		;Change the state to level 3
	sts sectorstate,r16
	ldi r16,EVAC_TONE			;Change the tone counter to the evac tone state
	sts tonecounter,r16
	call TriggeredText			;Update the text
	call ToneCheck				;Check which tone we are meant to play and turn on
LevelThreeLoop:
	call WaitingLoop			;The waiting loop which it will remain in until reset or isolated
	rjmp LevelThreeLoop

MultipleSectorsCheck:			;Function for the multiple sectors check 	
	cpi r25,5
	brsh LevelThree				;Any 2 sector tirggers will go straight to the last level
	cpi r25,3
	breq LevelThree
	ret

WaitingLoop:				;Remain in here until isolated or reset
	ldi r23,90					;Half a second
Waiting:
	call Timer0					;Same loop as level 1
	call StartADCConversion
	call TriggeredSectorOn
	call IsolatedSectorsCheck
	call ResetLevelCheck
	dec r23
	brne Waiting
	tst r26						;If no sectors are isolated then dont bother checking to flash
	breq WaitingEnd
	call IsolatedSectorFlash	;Flash any isolated sectors
WaitingEnd:
	ret


;-----------------------------------------------------------------------------------------------------------------------------------------------------------------
;-----------------------------------------------------------End Of Main Loop--------------------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------------------------------------------------------------------------------------


;-----------------------------------------------------------Helper Functions-------------------------------------------------------------------


;-----------------------------------------------------------Mask and Reset Functions-------------------------------------------------------------------

;Masks the SPI reset button and the ADC conversion reset button
ResetButtonMask:
	ldi r20,0x04 ; address of register GPINTENA (port A Interrupt on change)
	ldi r21,0b01010000 ; Interrupt on change for Emergency and Isoalte buttons
	call SPISendCommandChip
	ldi r16,ADC_RESET_TRIGGER
	sts adcresettrigger,r16
	ret

;Unmasks the SPI reset button and the ADC conversion reset button
ResetButtonUnmask:
	ldi r20,0x04 ; address of register GPINTENA (port A Interrupt on change)
	ldi r21,0b01110000 ; Interrupt on change for all buttons
	call SPISendCommandChip
	clr r16
	sts adcresettrigger,r16
	ret

;Stops the ADC conversion
StopADCConversion:
	ldi r16,(1<<ADEN)|(1<<ADSC)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADATE)
	sts ADCSRA,r16
	ret

;Starts the ADC conversion
StartADCConversion:
	ldi r16,(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADATE)
	sts ADCSRA,r16
	ret
	

SleepMode:					;Put the mega to sleep
	ldi r16,(1<<SE)|(1<<SM0) ;Sleep enable and ADC noise reduction mode set
	out SMCR,r16
	sleep					;Put to sleep
	clr r16					;Clear the Sleep enable so we don't unecessarily sleep again
	out SMCR,r16
	ret

;Reset all counters, triggers, constants, timers, states and button masks
ResetMode:		
	call AllSectorsOff ;Turn sectors off
	call ResetButtonUnmask	;Unmask reset buttons
	ldi r20,0x10	;address for register INTCAPA (Interrupt Captured Value)
	ldi r21,0b01110000 ; Clear any reading on the INT0 pin
	call SPIReadCommandChip
	ldi r16,(1<<PCINT23)|(1<<PCINT22)|(1<<PCINT21)		;Unmask all the secter pin interrupts
	sts PCMSK2,r16
	clr r26		;Reset isolated states
	clr r25		;Reset triggered states
	sts multipletriggers,r25		;Clear and reset all states
	ldi r16,RESETSTATE_1
	sts resetcounter,r16
	ldi r16,QUIESCENT_STATE
	sts sectorstate,r16
	ldi r16,ALERTSTATE_ON
	sts buzzer,r16
	sbi TIFR2,OCF2A				;Reset all flags, timer countings and counts
	clr r16
	sts tonecounter,r16
	sts TCCR1B,r16
	sts TCNT2,r16
	sts TCNT1H,r16
	sts TCNT1L,r16
	call StartADCConversion		;Begin ADC Conversions
	sei
	ret

;-------------------------------------------------------------------------Timer Functions-------------------------------------------------------------------

;Timer 1 used for the 2 second timer
TwoSecondTimer:
	push r16
	clr r16
	sts TCCR1B,r16 ;stop the clock while we initialize
	sts TCCR1A,r16 ;normal mode, WGM=0
	sts TCCR1C,r16 ;don't trigger the force output comparisons
	sts TIMSK1,r16
	ldi r16,HIGH(31250)		;Two Seconds
	sts OCR1BH,r16
	ldi r16,LOW(31250)
	sts OCR1BL,r16
	clr r16 ;counter will start counting at 0
	sts TCNT1H, r16
	sts TCNT1L, r16
	sbi	TIFR1,OCF1B ; turn on the output compare resgister flag
	ldi r16,(1<<OCIE1B)		;Output compare B interrupt for timer 1
	sts TIMSK1,r16
	ldi r16,(1<<CS10)|(1<<CS12)	;1024 prescaler and being counting
	sts TCCR1B,r16
	pop r16
	ret

;Timer 1 used for the buzzer
AlarmTimer:	
	push r16
	clr r16
	sts TCCR1B,r16			;Stop while we configure
	sts TIMSK1,r16			;Mask any compare interrupts
	ldi r16,(1 << COM1A0)	;Clear OC1A on compare match, CTC Mode
	sts TCCR1A,r16			
	clr r16
	sts TCCR1C,r16			
	lds r16,tonecounter		;If evacuate state then play evacuate sound otherwise paly alert sound
	cpi r16,EVAC_TONE
	breq SetEvacTone
	ldi r16,HIGH(Alert_Sound)
	sts OCR1AH,r16
	ldi r16,LOW(Alert_Sound)
	sts OCR1AL,r16
	rjmp AlarmTimerEnd
SetEvacTone:
	ldi r16,HIGH(Evac_Sound_One)
	sts OCR1AH,r16
	ldi r16,LOW(Evac_Sound_One)
	sts OCR1AL,r16
AlarmTimerEnd:
	clr r16 ;counter will start counting at 0
	sts TCNT1H,r16
	sts TCNT1L,r16
	ldi r16,(1 << CS10)|(1 << CS11)|(1 << WGM12)	;1024 prescaler and begin timer
	sts TCCR1B,r16
	pop r16
	ret

Timer0:
	push r16
	ldi r16,(1 << WGM01) ; CTC mode
	out TCCR0A,r16
	clr r16
	out TCCR0B,r16 ;stop the clock
	out TCNT0,r16
	ldi r16,0xff		;Max timer limit
	out OCR0A,r16
	sbi TIFR0,OCF0A		;clear compare output A flag
	ldi r16,(1 << CS02) | (1 << CS00)		;1024 precaler and being timer
	out TCCR0B,r16
Timerloop:
	sbis TIFR0,1		;loop until timer completes and clear output compare flag
	rjmp Timerloop
	sbi	TIFR0,OCF0A 
	pop r16
	ret

Nothing:

	ret

;-------------------------------------------------------------------------Sound Functions-------------------------------------------------------------------

;The "buzzer" byte is for changing the sound tones
;The "tone counter" byte is for what tone is playing

;Function to pause the alert sound
AlertToneOff:  
	ldi r16,ALERTSTATE_OFF
	sts buzzer,r16
	clr r16
	sts TCCR1B,r16
	ret
;Function to start the alert sound
AlertToneOn:
	ldi r16,ALERTSTATE_ON
	sts buzzer,r16
	ldi r16,(1 << CS10)|(1 << CS11)|(1 << WGM12)
	sts TCCR1B,r16
	ret

;Function to begin the alert sound. Timer2 will be used to change the sounds from on and off
AlertTone:
	ldi r16,ALERTSTATE_ON		;Start with alert tone on
	sts buzzer,r16
	ldi r16,ALERT_TONE			;Alert tone is set
	sts tonecounter,r16
	call AlarmTimer				;Start timer 1 and play the sound
	ldi r16,(1<<OCIE2A)			;Output compare A for timer 2
	sts TIMSK2,r16
	ldi r16,(1<<CS22)|(1<<CS21)|(1<<CS20)	;1024 prescaler and begin counting
	sts TCCR2B,r16	
	clr r28				;clear the reset the time loop constant
	ret

;Function to begin the evac sound. Timer2 will be used to change the sounds from high and low
EvacuateTone:
	ldi r16,EVAC_TONE_1
	sts buzzer,r16				;Start in evac tone 1
	call AlarmTimer
	ldi r16,(1<<OCIE2A)			;Same as above, start timer2 and the interrupt
	sts TIMSK2,r16
	ldi r16,(1<<CS22)|(1<<CS21)|(1<<CS20)
	sts TCCR2B,r16
	ret

;Interrupt for the tone changes
;This is a state machine that will check which tones need to be played and at what frequency
ToneChange:
	push r16
	in	r16,SREG
	push r16
	lds r16,buzzer
	cpi r16,ALERTSTATE_ON			;Check which tone we want to play
	breq TurnAlertOff
	cpi r16,ALERTSTATE_OFF
	breq TurnAlertOn
	cpi r16,EVAC_TONE_1
	breq EvacSoundOne
	cpi r16,EVAC_TONE_2
	breq EvacSoundTwo
	rjmp ToneChangeEnd
TurnAlertOff:
	cpi r28,35					;Run for approximately 1/6 of a second
	brsh ToneOff				;Check state
	rjmp ToneChangeEnd
ToneOff:						;clear counter, turn tone off
	clr r28
	call AlertToneOff			
	rjmp ToneChangeEnd
TurnAlertOn:
	cpi r28,15					;Run for 1/12 of a second
	brsh ToneOn					;Check state
	rjmp ToneChangeEnd
ToneOn:
	clr r28						;Clear counter and turn tone on
	call AlertToneOn
	rjmp ToneChangeEnd
EvacSoundOne:
	cpi r28,60					;Run for 1/3 of a second
	brsh ChangeToLowerTone		;Check state
	rjmp ToneChangeEnd
ChangeToLowerTone:
	ldi r16,HIGH(Evac_Sound_Two)		;Change the sound to a lower pitch and reset the timer counter, change the state and timer counter and the end
	ldi r17,LOW(Evac_Sound_Two)
	sts OCR1AH,r16
	sts OCR1AL,r17
	clr r16
	sts TCNT1H,r16
	sts TCNT1L,r16
	ldi r16,EVAC_TONE_2
	sts buzzer,r16
	clr r28
	rjmp ToneChangeEnd
EvacSoundTwo:
	cpi r28,60
	brsh ChangeToHigherTone
	rjmp ToneChangeEnd
ChangeToHigherTone:
	ldi r16,HIGH(Evac_Sound_One)		;Same as above
	ldi r17,LOW(Evac_Sound_One)
	sts OCR1AH,r16
	sts OCR1AL,r17
	clr r16
	sts TCNT1H,r16
	sts TCNT1L,r16
	ldi r16,EVAC_TONE_1
	sts buzzer,r16
	clr r28
ToneChangeEnd:
	pop r16
	out	SREG,r16
	pop r16
	inc r28
	reti

;-------------------------------------------------------------------------LED Functions-------------------------------------------------------------------


;Function to flash isolated sectors
;Isolated sectors are held in r26
IsolatedSectorFlash:
	mov r16,r26 ;Copy Isolated Register State
	lsl r16	;Shift left to align sector bits
	ori r16,0b00110000	;Set TWI bits
	out PORTC,r16	;Output to LEDS
	ldi r20,0x14 ;address for register OLATA (port A data output)
	mov r21,r26 ;Move the Triggered Sectors register to the mimic panel LEDS
	call SPISendCommandChip
	call Timer0					;wait a brief moment and turn sectors off
	call AllSectorsOff
	ret

;Function to turn triggerd sectors on
;Triggerd sectors are held in r25
TriggeredSectorOn:
	mov r16,r25	;Copy Triggered Register State
	lsl r16	;Shift left to align sector bits
	ori r16,0b00110000	;Set TWI bits
	out PORTC,r16	;Output to LEDS
	ldi r20,0x14 ;address for register OLATA (port A data output)
	mov r21,r25 ;Move the Triggered Sectors register to the mimic panel LEDS
	call SPISendCommandChip
	ret

;Function to turn all sectors off
AllSectorsOff:
	ldi r20,0x14 ;address for register OLATA (port A data output)
	ldi r17,0b00000000 ;All LEDs lit
	mov r21,r17 ;command to be sent
	call SPISendCommandChip
	ldi r16,0b00110000
	out PORTC,r16
	ret

;-------------------------------------------------------------------------Emergency Isolate Reset Functions-------------------------------------------------------------------

;External Interrupt for INT0
EmerIsoRes:
	push r16
	in	r16,SREG
	push r16
	cbi EIMSK,INT0 ;mask the interrupt so it doesn't trigger multiple times
	call CheckEmergency	;Check which button on the MCP was pressed 
	call CheckIsolate
	call CheckReset
	sbi EIMSK,INT0 ; turn the external interrupts back on
	pop r16
	out	SREG,r16
	pop	r16
	reti

CheckEmergency:
	ldi r20,0x12 ;address for register GPIOA (port A data input)
	call SPIReadCommandChip 
	andi r16,0b01000000
	sbrs r16,6				;Check if the Emergency button was pressed and call the function
	call Emergency
	ret

CheckReset:
	ldi r20,0x12 ;address for register GPIOA (port A data input)
	call SPIReadCommandChip 
	andi r16,0b00100000
	sbrs r16,5				;Check if the Reset button was pressed and call the function
	call Reset
	ret

CheckIsolate:
	ldi r20,0x12 ;address for register GPIOA (port A data input)
	call SPIReadCommandChip 
	andi r16,0b00010000
	sbrs r16,4				;Check if the Isolate button was pressed and call the function
	call Isolate
	ret


;Function for emergency. Sets the state and plays the tone
Emergency:
	ldi r16,EVAC_TONE		;Set sound state
	sts tonecounter,r16
	call ResetButtonUnmask		;Unmask the reset button
	ldi r16,RESETSTATE_1
	sts resetcounter,r16		;Reset the reset counter
	call TriggeredText			;Update the text
	call EvacuateTone			;Play the tone
	ret

;Function to isolate all triggered sectors. Will go to the quiescent state on the main code.
Isolate:
	or r25,r26 ;check if any sectors are already isolated
	mov r26,r25	;move all triggered secotors to isolated
	call MaskIsolatedButtons	;Mask the sectors that are isolated 
	clr r25						;Clear triggered sectors
	ldi r16,ISOLATED_STATE		;Set the isolated state
	sts isolatedtrigger,r16
	call ResetButtonUnmask		;Unmask the reset button
	ldi r16,RESETSTATE_1		;Reset the reset counter
	sts resetcounter,r16
	clr r16
	sts TCCR2B,r16			;Turn timer1 and timer2 off
	sts tonecounter,r16
	call AlertToneOff
	ret

;Function which checks r26 and masks the pins accordingly
MaskIsolatedButtons:
	ldi r16,0b11100000
	sbrc r26,0
	andi r16,0b01100000
	sbrc r26,1
	andi r16,0b10100000
	sbrc r26,2
	andi r16,0b11000000
	sts PCMSK2,r16
	ret

;Reset function, will check which reset state it is and branch accordingly. At the start it will begin the 2 second delay on timer1 and at the end will run 
;an interrupt which unmasks the reset button. In the case any other buttons are pushed, the reset button will unmask through their functions.
Reset:
	call ResetButtonMask		;Mask the reset button
	call AlertToneOff			;Turn the tones off
	call TwoSecondTimer			;Start the 2 second delay
	lds r16,resetcounter
	cpi r16,RESETSTATE_1		;Check which reset state we are in
	breq ResetTone
	cpi r16,RESETSTATE_2
	breq ResetTriggered
	cpi r16,RESETSTATE_3
	breq ResetIsolated
ResetEnd:
	ret
ResetTone:					;Reset the tone variables and update the text. On the main code this will return to the same place.
	clr r16
	sts tonecounter,r16
	ldi r16,ALERTSTATE_ON
	sts buzzer,r16
	ldi r16,RESETSTATE_2
	sts resetcounter,r16
	call TriggeredText
	rjmp ResetEnd
ResetTriggered:				;Reset the triggerd state register. This will return to the quiescent state
	ldi r16,RESETSTATE_3
	sts resetcounter,r16
	clr r25
	rjmp ResetEnd
ResetIsolated:				;Reset the isolated state register. This will return to the quiescent state where it will go to a full reset
	ldi r16,RESETSTATE_4
	sts resetcounter,r16
	clr r26
	rjmp ResetEnd

;The two second delay interrupt. Will trigger after 2 seconds of timer 1
TwoSecondDelay:
	clr r16
	sts TCCR1B,r16
	call ResetButtonUnmask
	reti

;-------------------------------------------------------------------------Sector Functions-------------------------------------------------------------------

;Sector check function
;1st checks which sector was triggered through r25 and then will check which level it is on.
;Depending on that continuously holding the button will not reset the corresponding timer value
SectorCheck:
	cpi r25,1
	breq Sector1ButtonCheck
	cpi r25,2
	breq Sector2ButtonCheck				;Check which sector is triggered
	cpi r25,4
	breq Sector3ButtonCheck
SectorCheckEnd:
	ret
Sector1ButtonCheck:
	lds r16,sectorstate
	cpi r16,SECTORSTATE_1
	breq Sector1LevelOne					;Checks which level it is on
	cpi r16,SECTORSTATE_2
	breq Sector1LevelTwo
Sector1LevelOne:	
	sbic PIND,7
	ldi r24,2
	rjmp SectorCheckEnd						;Depending on the level, it will reset the timer values that was set in the main code. This is the same for the rest of the function.
Sector1LevelTwo:
	sbic PIND,7
	ldi r24,6
	rjmp SectorCheckEnd
Sector2ButtonCheck:
	lds r16,sectorstate
	cpi r16,SECTORSTATE_1
	breq Sector2LevelOne
	cpi r16,SECTORSTATE_2
	breq Sector2LevelTwo
Sector2LevelOne:	
	sbic PIND,6
	ldi r24,2
	rjmp SectorCheckEnd
Sector2LevelTwo:
	sbic PIND,6
	ldi r24,6
	rjmp SectorCheckEnd
Sector3ButtonCheck:
	lds r16,sectorstate
	cpi r16,SECTORSTATE_1
	breq Sector3LevelOne
	cpi r16,SECTORSTATE_2
	breq Sector3LevelTwo
Sector3LevelOne:	
	sbic PIND,5
	ldi r24,2
	rjmp SectorCheckEnd
Sector3LevelTwo:
	sbic PIND,5
	ldi r24,6
	rjmp SectorCheckEnd


;External interrupt for the 3 sector buttons
SectorTrigger:
	push r16
	in	r16,SREG
	push r16
	call ResetButtonUnmask		;Unmask the reset button and the state
	ldi r16,RESETSTATE_1
	sts resetcounter,r16
	sbis PIND,5					;Depending on the button, set the sector.
	ori r25,0b00000100
	sbis PIND,6
	ori r25,0b00000010
	sbis PIND,7
	ori r25,0b00000001
	pop r16
	out	SREG,r16
	pop r16
reti	

;-------------------------------------------------------------------------ADC Conversion-------------------------------------------------------------------


;Voltage Divider values
;Emergency Vpin = 0 = 00 0000 0000 ADCH = 0
;Reset/Silence Vpin = 512 = 10 0000 0000 ADCH = 128
;Isolate Vpin = 682.67 = 10 1010 1010 ADCH = 170
;Nothing Vpin = 768 = 11 0000 0000 ADCH = 192

;Runs the ADC Conversion.
;I have found the conversion runs smoother when it runs less so i turn it off after every conversion and call it again in the timer loops. 
;If its in the Quiescent state then it remains on. 
ADCButton:
	push r16
	in	r16,SREG
	push r16
	lds r16,sectorstate
	cpi r16,QUIESCENT_STATE				;Check if in the quiescent state and remain on. Otherwise stop and check which value was converted
	breq KeepConversionRunning
	call StopADCConversion
KeepConversionRunning:
	lds r16,ADCH
	cpi r16,60
	brlo ADCEmergency
	cpi r16,130
	brlo ADCReset							;Values to be compared with
	cpi r16,177
	brlo ADCIsolate
ADCEnd:
	pop r16
	out	SREG,r16
	pop	r16
	reti
ADCEmergency:										;Depending on the check, it will call its respective functions
	call Emergency
	rjmp ADCEnd
ADCReset:
	lds r16,adcresettrigger							;Sets the reset button mask which will unmask after 2 seconds or another button push.
	cpi r16,ADC_RESET_TRIGGER
	breq ADCResetEnd
	call Reset
ADCResetEnd:	
	rjmp ADCEnd
ADCIsolate:
	call Isolate
	rjmp ADCEnd
	

;-------------------------------------------------------------SPI Send/Read functions-----------------------------------------------------

;Code provided by Rex
;Each function will read and write to both chips

SPISendCommandChip:
cbi PORTB,2 ;Slave select needs to be low to send data
ldi r16,0x40  ;telling the arduino we are peforming a write command
call SPI_SendByte
mov r16,r20 ;address of the register in the MCP
call SPI_SendByte
mov r16,r21 ;what value we are sending to the register
call SPI_SendByte
sbi PORTB,2 ; Slave select back to high so no more data is send
ret

SPIReadCommandChip:
cbi PORTB,2 ;Slave select needs to be low to send data
ldi r16,0x41 ;telling the arduino we are peforming a read command
call SPI_SendByte
mov r16,r20 ;address of the register in the MCP
call SPI_SendByte
mov r16,r21 ;what value the register is returning
call SPI_SendByte
sbi PORTB,2 ; and SS back high
ret

;
; Send one SPI byte (Returned data in r16)
;
SPI_SendByte:
out SPDR,r16 ;write to the SPDR register, this automatically sends the data to the MCP chip
SPI_wait:
in r16,SPSR ;once the data is sent the SPIF bit will be set
sbrs r16,SPIF
rjmp SPI_wait
in r16,SPDR ;read the data that was returned
ret

;-----------------------------------------------------------------------USART Functions-------------------------------------------------------

;Code to display to the serial monitor like putty
;I have created pointers and string buffers

;1st line will display the tone we are playing and the reset level depicted by "R"
;2nd line will display either isolated or triggered below its sector and then the level we are on. 

;"X" means triggered.
;"I" means isolated.

Str_NL:
	.db	0x0a, 0x0d, 0x00, 0x00

StringFixedFirstLine: .db "S1 S2 S3       R"
StringFixedSecondLine: .db " -- -- -- QUIES "	

StringTriggered: .db "X"
StringIsolated: .db "I"
StringAlert: .db "ALERT"
StringEvac: .db "EVAC "
StringQuiet: .db "QUIET"
StringLevOne: .db "LEV 1"
StringLevTwo: .db "LEV 2"
StringLevThree: .db "LEV 3"
StringQuies: .db "QUIES"
StringBlanks: .db " -- -- --"
StringResetOne: .db "1"
StringResetTwo: .db "2"
StringReset: .db "R"
	.dseg
LCD_line1: .byte 16
LCD_line2: .byte 16
	.cseg

;Inital text to load
TextInitialise:
	ldi ZL,LOW(StringFixedFirstLine*2)
	ldi ZH,HIGH(StringFixedFirstLine*2)
	ldi XL,LOW(LCD_line1)
	ldi XH,HIGH(LCD_line1)
	ldi r17,16						;How many characters to send
	call SendMessageToRAM			;Function to send the text to RAM
	ldi ZL,LOW(StringFixedSecondLine*2)
	ldi ZH,HIGH(StringFixedSecondLine*2)
	ldi XL,LOW(LCD_line2)
	ldi XH,HIGH(LCD_line2)
	ldi r17,16
	call SendMessageToRAM
	ret
SendMessageToRAM:				;Function to send the text to RAM. Provide by Gerry
	lpm r16,Z+
	st X+,r16					;Stores the string into the data segment pointed by LCD_Line1/2
	dec r17
	brne SendMessageToRAM
	ret

;Main text function to display information. It will check every state through the 1st 5 functions which will create the buffer. The text will then make a new line and display.
TriggeredText:
	call TextSectorState		;X, I or blank
	call TextSoundState			;Quiet, Alert or Evac
	call TextLevelState			;Quies, Lev 1, Lev 2, Lev 3
	call TextResetState			;1,2 or R
	call NewLineReturnCharacter	;Function for new line
	ldi ZL,LOW(LCD_line1)		;String pointers
	ldi ZH,HIGH(LCD_line1)		
	ldi r17,16					;How many characters
	call puts					;Print to serial via USART
	call NewLineReturnCharacter
	ldi ZL,LOW(LCD_line2)
	ldi ZH,HIGH(LCD_line2)
	ldi r17,16
	call puts
	call NewLineReturnCharacter
	call NewLineReturnCharacter
	ret

;These remaining function are buffers and state machines for the buffers that are identical to the state machines during the main code
;Ive just commented on the 1st one

;The reason for the pop and pushes of r26 was because I encountered this very strange bug where creating the buffers would cause r26. I narrowed it down to this part and the
;pop and pushing fixes the problem so who knows what it was.

StringSectorOneTriggered:
	push r26
	ldi ZL,LOW(StringTriggered*2)	;String to write
	ldi ZH,HIGH(StringTriggered*2)
	ldi XL,LOW(LCD_line2)		;Byte to write to
	ldi XH,HIGH(LCD_line2)
	ldi r17,1			;1 character
	call SendMessageToRAM	;Store to RAM
	pop r26
	ret
StringSectorTwoTriggered:
	push r26
	ldi ZL,LOW(StringTriggered*2)
	ldi ZH,HIGH(StringTriggered*2)
	ldi XL,LOW(LCD_line2+3)
	ldi XH,HIGH(LCD_line2+3)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringSectorThreeTriggered:
	push r26
	ldi ZL,LOW(StringTriggered*2)
	ldi ZH,HIGH(StringTriggered*2)
	ldi XL,LOW(LCD_line2+6)
	ldi XH,HIGH(LCD_line2+6)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringSectorOneIsolated:
	push r26
	ldi ZL,LOW(StringIsolated*2)
	ldi ZH,HIGH(StringIsolated*2)
	ldi XL,LOW(LCD_line2)
	ldi XH,HIGH(LCD_line2)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringSectorTwoIsolated:
	push r26
	ldi ZL,LOW(StringIsolated*2)
	ldi ZH,HIGH(StringIsolated*2)
	ldi XL,LOW(LCD_line2+3)
	ldi XH,HIGH(LCD_line2+3)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringSectorThreeIsolated:
	push r26
	ldi ZL,LOW(StringIsolated*2)
	ldi ZH,HIGH(StringIsolated*2)
	ldi XL,LOW(LCD_line2+6)
	ldi XH,HIGH(LCD_line2+6)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringBlankSectors:
	push r26
	ldi ZL,LOW(StringBlanks*2)
	ldi ZH,HIGH(StringBlanks*2)
	ldi XL,LOW(LCD_line2)
	ldi XH,HIGH(LCD_line2)
	ldi r17,9
	call SendMessageToRAM
	pop r26
	ret

TextResetState:					;Reset text buffer and state machine
	lds r16,resetcounter
	cpi r16,RESETSTATE_2
	breq StringResetLevelOne
	cpi r16,RESETSTATE_3
	breq StringResetLevelTwo
	push r26
	ldi ZL,LOW(StringReset*2)
	ldi ZH,HIGH(StringReset*2)
	ldi XL,LOW(LCD_line1+15)
	ldi XH,HIGH(LCD_line1+15)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringResetLevelOne:
	push r26
	ldi ZL,LOW(StringResetOne*2)
	ldi ZH,HIGH(StringResetOne*2)
	ldi XL,LOW(LCD_line1+15)
	ldi XH,HIGH(LCD_line1+15)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret
StringResetLevelTwo:
	push r26
	ldi ZL,LOW(StringResetTwo*2)
	ldi ZH,HIGH(StringResetTwo*2)
	ldi XL,LOW(LCD_line1+15)
	ldi XH,HIGH(LCD_line1+15)
	ldi r17,1
	call SendMessageToRAM
	pop r26
	ret


TextSoundState:					;Sound text buffer and state machine
	lds r16,tonecounter
	cpi r16,ALERT_TONE
	breq StringAlertSound
	cpi r16,EVAC_TONE
	breq StringEvacSound
StringNoSound:
	push r26
	ldi ZL,LOW(StringQuiet*2)
	ldi ZH,HIGH(StringQuiet*2)
	ldi XL,LOW(LCD_line1+9)
	ldi XH,HIGH(LCD_line1+9)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
StringAlertSound:
	push r26
	ldi ZL,LOW(StringAlert*2)
	ldi ZH,HIGH(StringAlert*2)
	ldi XL,LOW(LCD_line1+9)
	ldi XH,HIGH(LCD_line1+9)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
StringEvacSound:
	push r26
	ldi ZL,LOW(StringEvac*2)
	ldi ZH,HIGH(StringEvac*2)
	ldi XL,LOW(LCD_line1+9)
	ldi XH,HIGH(LCD_line1+9)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret

TextLevelState:					;Level text buffer and state machine
	lds r16,sectorstate
	cpi r16,SECTORSTATE_1
	breq StringLevelOne
	cpi r16,SECTORSTATE_2
	breq StringLevelTwo
	cpi r16,SECTORSTATE_3
	breq StringLevelThree
	push r26
	ldi ZL,LOW(StringQuies*2)
	ldi ZH,HIGH(StringQuies*2)
	ldi XL,LOW(LCD_line2+10)
	ldi XH,HIGH(LCD_line2+10)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
StringLevelOne:
	push r26
	ldi ZL,LOW(StringLevOne*2)
	ldi ZH,HIGH(StringLevOne*2)
	ldi XL,LOW(LCD_line2+10)
	ldi XH,HIGH(LCD_line2+10)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
StringLevelTwo:
	push r26
	ldi ZL,LOW(StringLevTwo*2)
	ldi ZH,HIGH(StringLevTwo*2)
	ldi XL,LOW(LCD_line2+10)
	ldi XH,HIGH(LCD_line2+10)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
StringLevelThree:
	push r26
	ldi ZL,LOW(StringLevThree*2)
	ldi ZH,HIGH(StringLevThree*2)
	ldi XL,LOW(LCD_line2+10)
	ldi XH,HIGH(LCD_line2+10)
	ldi r17,5
	call SendMessageToRAM
	pop r26
	ret
TextSectorState:
	call StringBlankSectors
	sbrc r25,0
	call StringSectorOneTriggered
	sbrc r25,1
	call StringSectorTwoTriggered
	sbrc r25,2
	call StringSectorThreeTriggered
	sbrc r26,0
	call StringSectorOneIsolated
	sbrc r26,1
	call StringSectorTwoIsolated
	sbrc r26,2
	call StringSectorThreeIsolated
	ret

NewLineReturnCharacter:  ;Character return and new line
	ldi ZL,LOW(Str_NL*2)
	ldi ZH,HIGH(Str_NL*2)
	call putSpace
	ret

;Code provided by Rex. I edited the "puts" to load from data instead of program memory and then created another one that loads from memory for the new line character return

putSpace:
	push	r16
putSpaceLoop:
	lpm	r16,Z+			;Load from memory
	tst	r16
	breq	putSpaceEnd
	call	putc	; send the character in r16
	rjmp	putSpaceLoop	; and loop until end of string
putSpaceEnd:			; finished the string
	pop	r16
	ret

puts:
	push	r16
puts0:
	ld	r16,Z+			;Load from data
	cpi	r17,0
	breq	puts1
	call	putc	; send the character in r16
	dec r17			;How many characters to send
	rjmp	puts0	; and loop until end of string
puts1:			; finished the string
	pop	r16
	ret


putc:
	push	r17
putc0:
	lds	r17,UCSR0A
	andi	r17,0x20	; check if there is space to put the character in the buffer
	breq	putc0
	sts	UDR0,r16
	pop	r17
	ret
