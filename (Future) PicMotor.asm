;==========================================================
; botmotor.asm
;----------------------------------------------------------
; Sample stepping motor support code culled from the first 
; PicBot Prototype.  See that code for supporting timer and
; setup code used to support the following code.
;----------------------------------------------------------
; Current capabilities:	(See documentation)
;==========================================================


;;; From section . . .

;==========================================================
;PAGE 0 -- The Main Program Page (Reset/power-up vector)
;----------------------------------------------------------

;Reset vector .............................................
	ORG	h'0000'
;.........................................................;

;;; . . .

;----------------------------------------------------------
;Command Handler

	snefl	RXCHAR,'M'	;Motor command?
	goto	CmdMotor	;Yes, do it

;;; . . .

;Motor command ----------------------------------
CmdMotor
	SCall	RxGetHex	;Get hex value (2-bytes)
	movfw	RXCHAR		;Store status data
	movwf	MOTSTAT
	SCall	MotorDelay0	;Get motor0 speed delay
	movwf	TMRMOT0		;Set motor0 timer
	movfw	MOTSTAT
	SCall	MotorDelay1	;Get motor1 speed delay
	movwf	TMRMOT1		;Set motor1 timer
	goto	CmdValid	;Done

;----------------------------------------------------------
;End of Page0
;==========================================================

;==========================================================
;PAGE 1 -- The Subroutines Page
;----------------------------------------------------------
                ORG     h'0800'
;..........................................................

;;; . . .

;----------------------------------------------------------
;Initialize PIC

	clrf	MOTSTAT		;Motor

	clrf	TMRMOT0		;Motor 0 step delay
	clrf	TMRMOT1		;Motor 1 step delay


;;; . . .

;----------------------------------------------------------
;Return motor speed delay (W)

MotorDelay1
	movwf	SELECT		;Swap nibbles
	swapf	SELECT,W

MotorDelay0
	movwf	SELECT		;Save speed index
	movlw	HIGH MotorDelay0
	movwf	PCLATH		;Set PCH
	movfw	SELECT		;Restore speed index
	andlw	h'07'		;3 lower bits=speed
	addwf	PCL,F		;Select delay value
	retlw	d'0'		;0=Off
	retlw	d'255'		;1=Slowest
	retlw	d'127'		;2
	retlw	d'63'		;3
	retlw	d'31'		;4
	retlw	d'15'		;5
	retlw	d'7'		;6
	retlw	d'3'		;7=Fastest

;----------------------------------------------------------
;Return motor phase for step index 0-3 (W)

MotorPhase
	movwf	SELECT		;Save step index
	movlw	HIGH MotorPhase	;Set PCH
	movwf	PCLATH
	movfw	SELECT		;Restore step index
	andlw	h'03'		;Limit to 0-3
	addwf	PCL,F		;Select phase bit pattern
	retlw	MOTPHASEA
	retlw	MOTPHASEB
	retlw	MOTPHASEC
	retlw	MOTPHASED

;----------------------------------------------------------
;Motor0 stepper

MotorStep0
	movfw	MOTSTAT		;Get motor0 step delay
	call	MotorDelay0
	movwf	TMRMOT0		;Set motor0 timer
	btfss	MOTSTAT,FWD0	;Run forward?
	goto	MotorStep0R	;No, reverse
	
MotorStep0F
	incf	MOTSTEP0,W	;Next step
	goto	MotorStep0Set	;Continue
	
MotorStep0R
	decf	MOTSTEP0,W	;Previous step
	
MotorStep0Set
	andlw	h'03'		;Force range=0-3
	movwf	MOTSTEP0
	goto	MotorStepGo	;Step motor(s)
				
;Motor1 stepper -------------------------------------------

MotorStep1
	movfw	MOTSTAT		;Get motor1 step delay
	call	MotorDelay1
	movwf	TMRMOT1		;Set motor1 timer
	btfss	MOTSTAT,FWD1	;Run forward?
	goto	MotorStep1R	;No, reverse

MotorStep1F
	incf	MOTSTEP1,W	;Next step
	goto	MotorStep1Set	;Continue
	
MotorStep1R
	decf	MOTSTEP1,W	;Previous step
	
MotorStep1Set
	andlw	h'03'		;Force range=0-3
	movwf	MOTSTEP1

;Output bits to set both motors --------------------------

MotorStepGo
	movfw	MOTSTEP1	;Get motor1 step index
	call	MotorPhase	;Get phase bits
	movwf	MOTTEMP		;Save result
	swapf	MOTTEMP,F	;Swap to high nibble
	movfw	MOTSTEP0	;Get motor0 step index
	call	MotorPhase	;Get phase bits
	iorwf	MOTTEMP,W	;OR nibbles together
	movwf	PORTD		;Output to Port D
	return

;;; . . .

;----------------------------------------------------------
;End of Page1
;==========================================================

;==========================================================
;PAGE 2 -- The Interrupt Handlers Page
;----------------------------------------------------------
                ORG     h'1000'
;..........................................................

;;; . . .

;----------------------------------------------------------
;Timer2 O/F Handler
; Timer2 overflow, occurs every 50 uSecs (.05 mS)

;;; . . .

TMR2IF_Motor0
	movfw	TMRMOT0		;Motor0 inactive?
	btfsc	STATUS,Z	;(Zero?)
	goto	TMR2IF_Motor1	;Yes, skip	
	decfsz	TMRMOT0,F	;No, decrement timer. Zero?
	goto	TMR2IF_Motor1	;No, skip
	ISCall	MotorStep0	;Yes, step motor0
	
TMR2IF_Motor1
	movfw	TMRMOT1		;Motor1 inactive?
	btfsc	STATUS,Z	;(Zero?)
	goto	TMR2IF_TSec	;Yes, skip	
	decfsz	TMRMOT1,F	;No, decrement timer. Zero?
	goto	TMR2IF_TSec	;No, skip
	ISCall	MotorStep1	;Yes, step motor1

;;; . . .


;----------------------------------------------------------
;End of Page2
;==========================================================

;==========================================================
;PAGE 3 -- Available (up to 1E00, which is the bootloader)
;----------------------------------------------------------
;               ORG     h'1800'
;..........................................................


;----------------------------------------------------------
;End of Page3
;==========================================================

;==========================================================
                END		;End of program
;==========================================================
