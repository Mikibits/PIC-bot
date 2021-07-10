;==========================================================
; Picbot.asm
;----------------------------------------------------------
; First Bot prototype project, an autonomous mobile robot 
; platform employing various features of the Motorola 
; PIC 16F877 microcontroller.
; Author:  Miki Marshall
; Updated: 2018.05.29
;----------------------------------------------------------
; Current capabilities:	(See documentation)
;==========================================================

;----------------------------------------------------------
;Compile options

	title		"PicBot 1"
	list 		p=16f877
	
	;Include files
	include	 	"p16f877.inc"
	include	 	"macros.inc"
	include		"register.inc"

	;Turn off annoying messages
	ERRORLEVEL	-306, -302, -202

;----------------------------------------------------------
;Configuration Bits

 __CONFIG	_CP_OFF&_WDT_OFF&_BODEN_OFF&_PWRTE_ON&_HS_OSC&_WRT_ENABLE_ON&_LVP_ON&_DEBUG_OFF&_CPD_OFF

;==========================================================

;==========================================================
;PAGE 0 -- The Main Program Page (Reset/power-up vector)
;----------------------------------------------------------

;Reset vector .............................................
	ORG	h'0000'
;.........................................................;

Reset
	clrf	STATUS		;Bank0
	clrf	PCLATH		;Page0
	goto	MainInit	;Main program

;Interrupt vector .........................................
	ORG	h'0004'
;..........................................................

Interrupt
	movwf	SAVINTW		;Save W register
	swapf	STATUS,W		;Save STATUS register
	clrf	STATUS		;(Bank 0)
	movwf	SAVINTSTAT
	movfw	PCLATH		;Save PCLATH register
	movwf	SAVINTPCL
	movfw	FSR		;Save FSR register
	movwf	SAVINTFSR
	movfw	SELECT		;Save lookup selection
	movwf	SAVSELECT
	bcf	PCLATH,3	;Page 2 (Interrupts)
	bsf	PCLATH,4
	goto	IntHandler	;Handle interrupt
	
Resume
	movfw	SAVSELECT	;Restore lookup selection
	movwf	SELECT
	movfw	SAVINTFSR	;Restore FSR register
	movwf	FSR      
	movfw	SAVINTPCL	;Restore PCLATH register
	movwf	PCLATH	
	swapf	SAVINTSTAT,W	;Restore STATUS/Bank reg.
	movwf	STATUS      
	swapf	SAVINTW,F	;Get saved W register
	swapf	SAVINTW,W	;Swap to self (sets status)
	retfie			;Return from interrupt

;----------------------------------------------------------
;Main Program

MainInit
	SCall	Initialize	;Setup PIC
	movlw	'P'		;Send "PIC Reset" status
	SCall	TxPut
	movlw	'R'
	SCall	TxPut
	SCall	TxCRLF

MainLoop
	btfsc	RUNSTAT,RXCMD	;Command ready?
	call	CommandHandler
	btfsc	RUNSTAT,ERR	;Error logged?
	call	ErrorHandler
	btfsc	RUNSTAT,BRK	;User break?
	call	BreakHandler
	goto	MainLoop	;Loop

;----------------------------------------------------------
;Break handler

BreakHandler
	movlw	'B'		;Send <break> to SCI
	SCall	TxPut
	movlw	'K'
	SCall	TxPut
	SCall	TxCRLF	

	;;Place triggered test code here;;
;	movlw	'M'
;	SCall	RxPut
;	movlw	'0'
;	SCall	RxPut
;	movlw	'F'
;	SCall	RxPut
;	movlw	'3'
;	SCall	RxPut
;	movlw	CR
;	SCall	RxPut
		
;	movlw	'M'
;	SCall	RxPut
;	movlw	'1'
;	SCall	RxPut
;	movlw	'R'
;	SCall	RxPut
;	movlw	'1'
;	SCall	RxPut
;	movlw	CR
;	SCall	RxPut
		
	bcf	RUNSTAT,BRK	;Clear break
	return			;Return to idle loop

;----------------------------------------------------------
;Command Handler

CommandHandler
	SCall	RxGet		;Get first cmd char
	movfw	RXCHAR
	
	xorlw	CR		;Equal <CR>?
	btfsc	STATUS,Z
	goto	CmdValid	;Yes, "ACK" connection

	movlw	h'20'		;>= 20?
	subwf	RXCHAR,W
	btfss	STATUS,C
	goto	CmdDone		;No, skip it
	
	movfw	RXCHAR
	xorlw	'A'		;A/D command?
	btfsc	STATUS,Z
	goto	CmdAD		;Yes
	xorlw	'A'
	
	xorlw	'D'		;SPI (data IO) command?
	btfsc	STATUS,Z
	goto	CmdSPI		;Yes
	xorlw	'D'
	
	xorlw	'E'		;Error command?
	btfsc	STATUS,Z
	goto	CmdError	;Yes
	xorlw	'E'
	
	xorlw	'I'		;Input port command?
	btfsc	STATUS,Z
	goto	CmdInPort	;Yes
	xorlw	'I'
	
;	xorlw	'M'		;Motor command?
;	btfsc	STATUS,Z
;	goto	CmdMotor	;Yes
;	xorlw	'M'	

	xorlw	'O'		;Output port command?
	btfsc	STATUS,Z
	goto	CmdOutPort	;Yes
	xorlw	'O'
	
	xorlw	'P'		;PIC system command?
	btfsc	STATUS,Z
	goto	CmdPic		;Yes
	xorlw	'P'
	
	xorlw	'S'		;Servo command?
	btfsc	STATUS,Z
	goto	CmdServo	;Yes
	xorlw	'S'
	
	goto	CmdInvalid	;Bad command

;A/D port command -----------------------------------------
CmdAD
	SCall	RxGet		;Get port#
	
	movlw	'0'		;Valid (>=0)?
	subwf	RXCHAR,W
	btfss	STATUS,C
	goto	CmdADBad	;No, error
	
	movlw	'5'		;Valid (<5)?
	subwf	RXCHAR,W
	btfsc	STATUS,C
	goto	CmdADBad	;No, error
	
	movfw	RXCHAR		;Convert to A/D address
	SCall	AsciiToHex
	movwf	ADCH
	
	SCall	SendAD		;Send pin value
	goto	CmdValid
		
CmdADBad
	movlw	ERR_BADPORT	;Bad port#
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
	goto	CmdClear	;Abort

;Input port command ---------------------------------------
CmdInPort
	movlw	'I'		;Send port value (as hex)
	SCall	TxPut
	movfw	PORTB
	SCall	TxPutHex	
	SCall	TxCRLF
	goto	CmdValid
		
;Motor command ----------------------------------
;CmdMotor
;	SCall	RxGet		;Get next char
;	movfw	RXCHAR
;	
;	xorlw	'0'		;Motor 0?
;	btfsc	STATUS,Z	
;	goto	CmdMotor0	;Yes
;	xorlw	'0'
;	
;	xorlw	'1'		;Motor 1?
;	btfsc	STATUS,Z	
;	goto	CmdMotor1	;Yes
;	goto	CmdMotorBadNo	;Bad motor#
;	
;CmdMotor0
;	SCall	RxGet		;Get next char
;	movfw	RXCHAR
;	
;	xorlw	'F'		;Forward?
;	btfsc	STATUS,Z	
;	goto	CmdMotor0Fwd	;Yes
;	xorlw	'F'
;	
;	xorlw	'H'		;Hold position?
;	btfsc	STATUS,Z	
;	goto	CmdMotor0Hold	;Yes
;	xorlw	'H'
;	
;	xorlw	'R'		;Reverse?
;	btfsc	STATUS,Z	
;	goto	CmdMotor0Rev	;Yes
;	xorlw	'R'
;	
;	xorlw	'S'		;Stop?
;	btfsc	STATUS,Z	
;	goto	CmdMotor0Stop	;Yes
;	goto	CmdMotorBadDir	;Bad direction
;	
;CmdMotor0Fwd
;	bsf	RUNSTAT,M0FWD	;Set Motor0=Fwd
;	bsf	RUNSTAT,M0RUN
;	goto	CmdMotor0Speed
;	
;CmdMotor0Rev
;	bcf	RUNSTAT,M0FWD	;Set Motor0=Rev
;	bsf	RUNSTAT,M0RUN
;	goto	CmdMotor0Speed
;	
;CmdMotor0Hold
;	bcf	RUNSTAT,M0RUN	;Set Motor0=Stop
;	goto	CmdValid
;	
;CmdMotor0Stop
;	bcf	RUNSTAT,M0RUN	;Set Motor0=Stop
;	movlw	h'5'		;Set phase to "rest"
;	movwf	M0PHASE		
;	goto	CmdValid
;	
;CmdMotor0Speed
;	SCall	RxGet		;Get next char
;	movfw	RXCHAR
;
;	SCall	AsciiToHex	;Convert to hex value	
;	movwf	M0SPEED		;Store as speed
;	
;	movlw	'8'		;Valid (<8)?
;	subwf	RXCHAR,W
;	btfsc	STATUS,C
;	goto	CmdMotorBadSpd	;No, error	
;
;	SCall	Motor0Step	;Activate motor0
;	goto	CmdValid	;Done
;
;CmdMotor1
;	SCall	RxGet		;Get next char
;	movfw	RXCHAR
;	
;	xorlw	'F'		;Forward?
;	btfsc	STATUS,Z	
;	goto	CmdMotor1Fwd	;Yes
;	xorlw	'F'
;	
;	xorlw	'H'		;Hold position?
;	btfsc	STATUS,Z	
;	goto	CmdMotor1Hold	;Yes
;	xorlw	'H'
;	
;	xorlw	'R'		;Reverse?
;	btfsc	STATUS,Z	
;	goto	CmdMotor1Rev	;Yes
;	xorlw	'R'
;	
;	xorlw	'S'		;Stop?
;	btfsc	STATUS,Z	
;	goto	CmdMotor1Stop	;Yes
;	goto	CmdMotorBadDir	;Bad direction
;	
;CmdMotor1Fwd
;	bsf	RUNSTAT,M1FWD	;Set Motor1=Fwd
;	bsf	RUNSTAT,M1RUN
;	goto	CmdMotor1Speed
;	
;CmdMotor1Rev
;	bcf	RUNSTAT,M1FWD	;Set Motor1=Rev
;	bsf	RUNSTAT,M1RUN
;	goto	CmdMotor1Speed
;	
;CmdMotor1Hold
;	bcf	RUNSTAT,M1RUN	;Set Motor1=Stop
;	goto	CmdValid
;	
;CmdMotor1Stop
;	bcf	RUNSTAT,M1RUN	;Set Motor1=Stop
;	movlw	h'5'		;Set phase to "rest"
;	movwf	M1PHASE		
;	goto	CmdValid
;	
;CmdMotor1Speed
;	SCall	RxGet		;Get next char
;	movfw	RXCHAR
;	SCall	AsciiToHex	;Convert to hex value
;	movwf	M1SPEED		;Store as speed
;	
;	movlw	'8'		;Valid (<8)?
;	subwf	RXCHAR,W
;	btfsc	STATUS,C
;	goto	CmdMotorBadSpd	;No, error	
;
;	SCall	Motor1Step	;Activate motor1
;	goto	CmdValid	;Done
;	
;CmdMotorBadNo
;	movlw	ERR_BADMOTNO	;Bad motor#
;	movwf	ERRSTAT		;Log to error status
;	bsf	RUNSTAT,ERR	;Flag an error
;	goto	CmdClear	;Abort
;
;CmdMotorBadDir
;	movlw	ERR_BADMOTDIR	;Bad direction
;	movwf	ERRSTAT		;Log to error status
;	bsf	RUNSTAT,ERR	;Flag an error
;	goto	CmdClear	;Abort
;
;CmdMotorBadSpd
;	movlw	ERR_BADMOTSPD	;Bad speed
;	movwf	ERRSTAT		;Log to error status
;	bsf	RUNSTAT,ERR	;Flag an error
;	goto	CmdClear	;Abort

;Output port command --------------------------------------

	;NOTE:  If the above motor stuff works, this may be
	; superceded by it.  Output ports may have to be 
	; moved to remaining C and E ports...

CmdOutPort
	SCall	RxGetHex	;Get hex value
	movfw	RXCHAR		;Copy to Output port (D)
	movwf	PORTD
	goto	CmdValid

;System command -------------------------------------------
CmdPic
	SCall	RxGet		;Get next char
	movfw	RXCHAR
	
	xorlw	'B'		;Bootload command?
	btfsc	STATUS,Z
	goto	CmdPicBootload	;Yes
	xorlw	'B'
	
	xorlw	'R'		;Reset command?
	btfsc	STATUS,Z
	goto	CmdPicReset	;Yes
	xorlw	'R'
	
        xorlw	'Z'		;Sleep command?
        btfsc	STATUS,Z
	goto	CmdPicSleep	;Yes
	
	goto	CmdInvalid	;Invalid command

CmdPicBootload
	bsf	PCLATH,3	;Bootloader Page (4)
	bsf	PCLATH,4
	goto	h'1FFF'		;Goto bootloader entry point
        ;Bootloader should announce itself as "PB"

CmdPicReset
	goto	Reset		;Reset PIC
	;Should announce Reset with "PR"
	
CmdPicSleep
	sleep			;Put PIC to sleep
	goto	CmdValid	;Response to waking back up?

;Servo port command -----------------------------------------
CmdServo
	SCall	RxGet		;Get port#
	
	movlw	'0'		;Valid (>=0)?
	subwf	RXCHAR,W
	btfss	STATUS,C
	goto	CmdServoBad	;No, error
	
	movlw	'5'		;Valid (<5)?
	subwf	RXCHAR,W
	btfsc	STATUS,C
	goto	CmdServoBad	;No, error
	
	movfw	RXCHAR		;Convert to Servo address
	SCall	AsciiToHex
	addlw	SERVODELAY0
	movwf	SERVOTEMP	;Save it
	
	SCall	RxGetHex	;Get position setting
	
	movfw	RXCHAR		;Zero?
	btfsc	STATUS,Z	
	goto	CmdServoOff	;Yes, shutoff servo
	
	sublw	d'42'		;Valid range (0 - 2A)?
	btfss	STATUS,C
	goto	CmdServoPosBad	;No, error
	
	movlw	d'4'		;Tweak pulse range for 
	addwf	RXCHAR,F	;servo timing
	
CmdServoOff
	movfw	SERVOTEMP	;At saved servo port
	movwf	FSR
	movfw	RXCHAR		;Store position delay
	movwf	INDF
	
	goto	CmdValid	;Done

CmdServoBad
	movlw	ERR_BADPORT	;Bad port#
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
	goto	CmdClear	;Abort

CmdServoPosBad
	movlw	ERR_BADSRVPOS	;Bad port#
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
	goto	CmdClear	;Abort

;SPI command ----------------------------------------------
CmdSPI

	;;; To Do ...
	
	goto	CmdInvalid
	
;Error command --------------------------------------------
CmdError
	SCall	RxGet		;Get error no (0-Z)
	movfw	RXCHAR		;Move to error reg
	movwf	ERRSTAT
	bsf	RUNSTAT,RXERR	;Set error recvd status
	goto	CmdValid	;Done

;Command cleanup ................................
CmdInvalid
	movlw	ERR_BADCMD	;Invalid command
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
	goto	CmdClear	;Abort
	
CmdValid
	movlw	'O'		;Reply "OK"
	SCall	TxPut
	movlw	'K'
	SCall	TxPut
	SCall	TxCRLF
	
CmdClear
	btfsc	SCISTAT,RXEMPTY	;Chars left?
	goto	CmdDone		;No, done
	movfw	RXCHAR		;Yes, <CR>?
	xorlw	CR
	btfsc	STATUS,Z
	goto	CmdDone		;Yes, all done
	SCall	RxGet		;No, get next char
	goto	CmdClear	;Loop

CmdDone
	return			;Return to idle loop

;----------------------------------------------------------
;Error Handler

ErrorHandler
	movlw	'E'		;Send error
	SCall	TxPut	
	movfw	ERRSTAT
	SCall	TxPut
	SCall	TxCRLF
	
	bcf	RUNSTAT,ERR	;Clear error
	clrf	ERRSTAT
	
	return			;Return to idle loop
	
;----------------------------------------------------------
;End of Page0
;==========================================================

;==========================================================
;PAGE 1 -- The Subroutines Page
;----------------------------------------------------------
                ORG     h'0800'
;..........................................................

;----------------------------------------------------------
;Convert ASCII byte (W) to low-nibble hex value (W)

AsciiToHex
	addlw	h'BF'		;Add -'A'
	btfss	STATUS,C	;Positive?
	addlw	h'07'		;No, add 17 (0-9)
	addlw	h'0A'		;Yes, add 10 (A-F)
	return

;----------------------------------------------------------
;Retrieve the current servo delay

GetServoDelay
	addwf	PCL,F		;Select Servo delay
	goto	GetServo0	;Servo 0
	goto	GetServo1	;Servo 1
	goto	GetServo2	;Servo 2
	goto	GetServo3	;Servo 3
	goto	GetServo4	;Servo 4

GetServo0
	movfw	SERVODELAY0	;Servo 0
	return

GetServo1
	movfw	SERVODELAY1	;Servo 1
	return

GetServo2
	movfw	SERVODELAY2	;Servo 2
	return

GetServo3
	movfw	SERVODELAY3	;Servo 3
	return

GetServo4
	movfw	SERVODELAY4	;Servo 4
	return

;----------------------------------------------------------
;Convert nibble hex value (W) to ASCII byte (W)

HexToAsciiH
	movwf	SELECT		;High-nibble
	swapf	SELECT,W	;Move to low-nibble
	
HexToAscii
	movwf	SELECT		;Save table seletion
	movlw	HIGH HexToAscii	;Set PCH
	movwf	PCLATH
	movfw	SELECT		;Restore table selection
	andlw	h'0F'		;Use low-nibble only
	addwf	PCL,F		;Return correct value
	retlw	'0'
	retlw	'1'
	retlw	'2'
	retlw	'3'
	retlw	'4'
	retlw	'5'
	retlw	'6'
	retlw	'7'
	retlw	'8'
	retlw	'9'
	retlw	'A'
	retlw	'B'
	retlw	'C'
	retlw	'D'
	retlw	'E'
	retlw	'F'

;----------------------------------------------------------
;Initialize PIC

Initialize
	Bank0			;Default bank
	bcf	INTCON,GIE      ;Disable interrupts
	btfsc	INTCON,GIE      ;Did it take?
	goto	$-2		;No, try again
	
InitPorts			;Setup I/O ports
	clrf    PORTA		;Clear output latches  
	clrf    PORTB
	clrf    PORTC
	clrf    PORTD
	clrf    PORTE
	clrf    ADCON0		;A/D = standby
	Bank1
	movlw   b'00111111'	;A = All A/D (exc A4),
	movwf	TRISA
	movlw   b'00000010'	;left-justified
	movwf	ADCON1
	movlw   b'11111111'	;B = Digital inputs
	movwf	TRISB
	movlw   b'11000000'	;C = Digital outputs
	movwf	TRISC		;(except SCI/SPI)			
	movlw   b'00000000'	;D = Digital outputs
	movwf	TRISD
	movlw   b'00000000'	;E = Digital outputs
	movwf	TRISE
	Bank0
	clrf	CCP1CON		;CCP1 mode=off
	clrf	CCP2CON		;CCP2 mode=off

InitRegisters			;Clear status registers:
	clrf	RUNSTAT		;Runtime
	clrf	ERRSTAT		;Error
	clrf	SCISTAT		;Serial communication
	clrf	RUNSTAT		;Motor
	clrf	SERVODELAY0	;Servos pulse delays (0-5)
	clrf	SERVODELAY1
	clrf	SERVODELAY2
	clrf	SERVODELAY3
	clrf	SERVODELAY4
	clrf	RXCMDCNT	;Clear command counter
				;Clear timers:
	clrf	TMRHMSEC	;Hundredths of mSecs (10 uSecs)
	clrf	TMRMSEC		;Milliseconds (100 TMRHMSEC's)
	clrf	TMRTSEC		;Tenth seconds (100 TMRMSEC's)
	clrf	TMRAD		;A/D conversion timer
	clrf	TMRMOT0		;Motor 0 step delay
	clrf	TMRMOT1		;Motor 1 step delay
	clrf	SERVOTIMER	;Servo pulse timer (1-2ms)
	movlw	d'80'		;Next servo pulse timer
	movwf	SERVOTMRNEXT	;(80 * .05ms = 4ms)
	clrf	SERVOSELECT	;Select current servo (0-4)
				;(4ms * 5 servos = 20ms)
				
InitTimers			;Setup Tmr2=default/internal
	;Timer0 (and misc B-port stuff)
	Bank1
	movlw	b'11000000'	;B-Pullups=off, B0-edge=hi;
	movwf	OPTION_REG	;Tmr0=int. osc.,
				;Prescaler=Tmr0 @ 1:2
	;Timer1 (Not in use)
	Bank0
	clrf	T1CON		;Prescale=1:1, osc=off,
				;sync=off, Tmr1=int. osc.,
				;Tmr1=standby
	;Timer2 (Default timer)
	clrf	T2CON		;Pre/postscale=1:1, Standby
	Bank1
	movlw	d'250'		;TMR2 O/F's every .05 mSeconds
	movwf	PR2
	Bank0
	bcf	PIR1,TMR2IF	;Clear interrupt flag
	bsf	T2CON,TMR2ON	;Turn on Timer 2
	movlw	d'20'		;Set timers
	movwf	TMRHMSEC	; (20 HMSecs = 1 millisec)
	movlw	d'100'
	movwf	TMRMSEC		; (100 mSecs = 1 tenthSec)

InitSCI				;Setup SCI port
	Bank1
	movlw	d'129'		;BRGH=0, 2400 baud
	movwf	SPBRG		;BRGH=1, 9600 baud
	movlw	b'00100110'	;TX: On, Asynch, 8-bit
	movwf	TXSTA		;BRGH=1 (9600 baud)
	Bank0
	movlw	b'10010000'	;RC: On, Contin., 8-bit
	movwf	RCSTA			
	clrf	CNTR		;Use a counter...
	incfsz	CNTR,F		;To pause for the...
	goto	$-1		;Port to settle down
	movfw	RCREG		;Clear out any garbage
	movfw	RCREG
	movfw	RCREG
	bsf	SCISTAT,RXEMPTY	;Set buffers=empty
	Bank1
	movlw	RXFIFO		;Set Put=top
	movwf	RXPUTPTR
	movlw	RXFIFO		;Set Get=top
	movwf	RXGETPTR

InitSPI				;Setup SPI port
	;(stay in Bank2)
	clrf	SSPSTAT		;Clear status
	clrf	SSPCON2		;No I2C stuff	
	Bank0			
	movlw	b'00100000'	;Enable SPI @ 5 MHz
	movwf	SSPCON

InitInterrupts			;Setup default interrupts
	clrf	PIR1		;Clear interrupt flags
	bsf	PIR1,TXIF	;(except TXIF)
	clrf	PIR2
	Bank1
	movlw	b'00101010'	;RCIE,TMR2,SSPIE=ON
	movwf	PIE1		
	clrf	PIE2
	Bank0
	movlw	b'11010000'	;GIE,PEIE,INTE=ON
	movwf	INTCON
InitDone
	return

;----------------------------------------------------------
;Return motor speed delay (W)

;MotorDelay
;	addwf	PCL,F		;Select delay value
;	retlw	d'255'		;0=Slowest
;	retlw	d'127'		;1
;	retlw	d'63'		;2
;	retlw	d'31'		;3
;	retlw	d'15'		;4
;	retlw	d'7'		;5
;	retlw	d'3'		;6
;	retlw	d'1'		;7=Fastest
	
;----------------------------------------------------------
;Return motor phase (W)

;MotorPhase
;	addwf	PCL,F		;Select phase value
;	retlw	b'00001100'
;	retlw	b'00000110'
;	retlw	b'00000011'
;	retlw	b'00001001'
;	retlw	b'00000000'	;Motor resting
	

;----------------------------------------------------------
;Motor0 stepper

;Motor0Step
;	movfw	M0SPEED		;Convert speed to delay
;	call	MotorDelay
;	movwf	TMRMOT0		;Set motor0 timer
;	
;	btfss	RUNSTAT,M0FWD	;Run forward?
;	goto	Mot0Rev		;No, reverse
;	
;Mot0Fwd
;	incf	M0PHASE,F	;Increment phase#
;		
;	movlw	h'4'		;>= 4?
;	subwf	M0PHASE,W
;	btfsc	STATUS,C
;	clrf	M0PHASE		;Yes, reset to 0
;	goto	Mot0Done	;Done
;	
;Mot0Rev
;	movfw	M0PHASE		;Phase no = 0?
;	btfss	STATUS,Z
;	goto	Mot0RevA	;No
;	
;	movlw	h'3'		;Yes, set to 3
;	movwf	M0PHASE	
;	goto	Mot0Done	;Done
;	
;Mot0RevA
;	decf	M0PHASE,F	;Decrement phase no
;	
;Mot0Done
;	return
			
;Motor1 stepper -------------------------------------------

;Motor1Step
;	movfw	M1SPEED		;Convert speed to delay
;	call	MotorDelay
;	movwf	TMRMOT1		;Set motor1 timer
;	
;	btfss	RUNSTAT,M1FWD	;Run forward?
;	goto	Mot1Rev		;No, reverse
;	
;Mot1Fwd
;	incf	M1PHASE,F	;Increment phase#
;		
;	movlw	h'4'		;>= 4?
;	subwf	M1PHASE,W
;	btfsc	STATUS,C
;	clrf	M1PHASE		;Yes, reset to 0
;	goto	Mot1Done	;Done
;	
;Mot1Rev
;	movfw	M1PHASE		;Phase no = 0?
;	btfss	STATUS,Z
;	goto	Mot1RevA	;No
;	
;	movlw	h'3'		;Yes, set to 3
;	movwf	M1PHASE
;	goto	Mot1Done	;Done
;	
;Mot1RevA
;	decf	M1PHASE,F	;Decrement phase no
;	
;Mot1Done
;	return

; Motor step-setter (both motors) -------------------------

;MotorSet
;	clrf	MTEMP		;Clear, in case...
;	btfss	RUNSTAT,M1RUN	;Motor1 running?
;	goto	MotorSetA	;No, leave zeros
;	
;	movfw	M1PHASE		;Get motor1 phase
;	call	MotorPhase
;	
;	movwf	MTEMP		;Set to upper nibble
;	swapf	MTEMP,F
;	
;MotorSetA
;	clrw			;Clear, in case...
;	btfss	RUNSTAT,M0RUN	;Motor0 running?
;	goto	MotorSetB	;No, leave zeros
;	
;	movfw	M0PHASE		;Get motor0 phase
;	call	MotorPhase
;	
;MotorSetB
;	iorwf	MTEMP,W		;Combine as lower nibble
;	
;	movwf	PORTD		;Update motor port
;	return

;----------------------------------------------------------
;Set the current servo pin (from pin # in W)

PulseServo
	addwf	PCL,F		;Pulse selected servo pin
	goto	PulseSrv0	;Servo 0
	goto	PulseSrv1	;Servo 1
	goto	PulseSrv2	;Servo 2
	goto	PulseSrv3	;Servo 3
	goto	PulseSrv4	;Servo 4
	
PulseSrv0
	bsf	PORTE,0		;Pin E0 on
	goto	PulseSrvDone
	
PulseSrv1
	bsf	PORTE,1		;Pin E1 on
	goto	PulseSrvDone
	
PulseSrv2
	bsf	PORTE,2		;Pin E2 on
	goto	PulseSrvDone
	
PulseSrv3
	bsf	PORTC,1		;Pin C1 on
	goto	PulseSrvDone
	
PulseSrv4
	bsf	PORTC,2		;Pin C2 on

PulseSrvDone
	return

;----------------------------------------------------------
;Get a character from SCI RX buffer (returns RXCHAR)

RxGet
	bcf	INTCON,GIE	;Disable interrupts
	btfsc	INTCON,GIE	;Did it take?
	goto	$-2		;No, try again
	
	Bank1			;(FIFO bank)
	clrf	RXCHAR		;Clear result reg				
	btfsc	SCISTAT,RXEMPTY	;FIFO empty?
	goto	RxGetError	;Yes, error

	movfw	RXGETPTR		;Get char from FIFO
	movwf	FSR
	movfw	INDF
	movwf	RXCHAR

	xorlw	CR		;<CR>?
	btfsc	STATUS,Z	
	decf	RXCMDCNT,F	;Yes, decr. command counter
	
	incf	RXGETPTR,F	;Increment Get pointer
	
	movfw	RXGETPTR		;Past end of buffer?
	sublw	RXFIFOMAX
	btfsc	STATUS,C
	goto	RxGetDone	;No, done

	movlw	RXFIFO		;Set get=top
	movwf	RXGETPTR
	goto	RxGetDone	;Done

RxGetError
	movlw	ERR_RXGETUF	;Log underflow error
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error

RxGetDone
	call	RxStatSet	;Update FIFO statuses
	bsf	INTCON,GIE	;Enable interrupts
	Bank0			;(Default bank)
	return
	
;----------------------------------------------------------
;Get 2 ascii hex chars, return 1-byte hex value (RXCHAR)

RxGetHex
        call	RxGet		;Get next char
        movfw	RXCHAR
        
	call	AsciiToHex	;Convert to hex nibble

        movwf	HEXBYTE		;Swap nibble to high position
        swapf	HEXBYTE,F

        call	RxGet		;Get next char
        movfw	RXCHAR
        
	call	AsciiToHex	;Convert to hex nibble
        
        iorwf	HEXBYTE,F       ;Combine nibbles
        movfw	HEXBYTE
        movwf	RXCHAR		;Return in RXCHAR
        return

;----------------------------------------------------------
;Put a character into SCI RX buffer (from W)

RxPut
	movwf	RXTEMP		;Save char
	
	Bank1			;(FIFO bank)
	btfsc	SCISTAT,RXFULL	;Already full?
	goto	RxPutError	;Yes, log an error
	
	movfw	RXTEMP		;<CR>?
	xorlw	h'0D'
	btfsc	STATUS,Z
	incf	RXCMDCNT,F	;Yes, incr. command counter
	
	movfw	RXPUTPTR		;Address (ind.) Put pointer
	movwf	FSR
	movfw	RXTEMP		;Restore char
	movwf	INDF		;Insert in FIFO
	
	incf	RXPUTPTR,F	;Increment Put pointer
	
	movfw	RXPUTPTR		;Past end of buffer?
	sublw	RXFIFOMAX
	btfsc	STATUS,C	
	goto	RxPutDone	;No, done
	
	movlw	RXFIFO		;Yes, set put=top
	movwf	RXPUTPTR
	goto	RxPutDone	;Done

RxPutError
	movlw	ERR_RXPUTOF	;Overflow error
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
				
RxPutDone
	call	RxStatSet	;Update FIFO statuses
	Bank0			;(Default bank)
	return

;----------------------------------------------------------
;Determine RXFIFO Full & Empty statuses

RxStatSet
	Bank1			;(RXFIFO bank)
	bcf	SCISTAT,RXEMPTY	;Clear empty flag
	
	movfw	RXGETPTR		;Get=Put positions?
	subwf	RXPUTPTR,W
	btfsc	STATUS,Z	
	bsf	SCISTAT,RXEMPTY	;Yes, set empty flag
	
	bcf	SCISTAT,RXFULL	;Clear full flag
	
	movfw	RXGETPTR		;Get at top of buffer?
	xorlw	RXFIFO
	btfss	STATUS,Z		
	goto	RxStatSet2	;No, continue
	
	movfw	RXPUTPTR		;Put at end of buffer?
	xorlw	RXFIFOMAX
	btfsc	STATUS,Z
	bsf	SCISTAT,RXFULL	;Yes, set full flag

RxStatSet2
	decf	RXGETPTR,W	;Look at get-1
	
	xorwf	RXPUTPTR,W	;Compare w/put
	btfsc	STATUS,Z		;(Get-1)=put?
	bsf	SCISTAT,RXFULL	;Yes, set full flag
	
	bcf	RUNSTAT,RXCMD	;Clear RX cmd flag
	
	movfw	RXCMDCNT		;RX commands=0?
	btfss	STATUS,Z
	bsf	RUNSTAT,RXCMD	;No, set RX cmd flag
	
	Bank0			;(Default bank)				
	return

;----------------------------------------------------------
;Send A/D value for channel ADCH	

SendAD
	clrf	ADCON0		;Clear A/D contr
	bsf	ADCON0,ADCS1	;Clock=FOsc/32
	btfsc	ADCH,2		;Set channel
	bsf	ADCON0,CHS2
	btfsc	ADCH,1
	bsf	ADCON0,CHS1
	btfsc	ADCH,0
	bsf	ADCON0,CHS0
	bsf	ADCON0,ADON	;Turn on A/D module
	
	movlw	h'02'		;Wait 2 msecs to acquire
	movwf	TMRAD

SendADWait
	movfw	TMRAD		;Zero?
	btfss	STATUS,Z
	goto	SendADWait	;No, wait some more
	
	bsf	ADCON0,GO_DONE	;Yes, start conversion

SendADGo
	btfsc	ADCON0,GO_DONE	;Done?
	goto	SendADGo		;No, wait
	
	movfw	ADRESH		;Send as ascii hex chars
	call	TxPutHex
	call	TxCRLF

	bcf	ADCON0,ADON	;Turn off A/D module
	return

;----------------------------------------------------------
;Send a line terminator

TxCRLF
	movlw	CR		;Send <CR><LF>
	call	TxPut
	movlw	LF
	call	TxPut
	return

;----------------------------------------------------------
;Send a character to the SCI

TxPut
	bcf	INTCON,GIE	;Disable interrupts
	btfsc	INTCON,GIE	;Did it take?
	goto	$-2		;No, try again
	btfss	PIR1,TXIF       ;Buffer empty?
	goto	$-1		;No, try again
	movwf	TXREG           ;Transmit byte
	bsf	INTCON,GIE	;Enable interrupts
	return

;----------------------------------------------------------
;Send 1-byte hex value (W) as 2-byte ascii hex chars

TxPutHex
	movwf	HEXBYTE		;Save original value
	
	call	HexToAsciiH	;Send high nibble
	call	TxPut
	
	movfw	HEXBYTE		;Retrieve original value
	
	call	HexToAscii	;Send low nibble
	call	TxPut
	return

;----------------------------------------------------------
;Reset the current servo pin

UnpulseServo
	addwf	PCL,F		;Unpulse selected servo pin
	goto	UnpulseSrv0	;Servo 0
	goto	UnpulseSrv1	;Servo 1
	goto	UnpulseSrv2	;Servo 2
	goto	UnpulseSrv3	;Servo 3
	goto	UnpulseSrv4	;Servo 4
	
UnpulseSrv0
	bcf	PORTE,0		;Pin E0-2 off
	goto	UnpulseSrvDone
	
UnpulseSrv1
	bcf	PORTE,1		;Pin E1 off
	goto	UnpulseSrvDone
	
UnpulseSrv2
	bcf	PORTE,2		;Pin E2 off
	goto	UnpulseSrvDone
	
UnpulseSrv3
	bcf	PORTC,1		;Pin C1 off
	goto	UnpulseSrvDone
	
UnpulseSrv4
	bcf	PORTC,2		;Pin C2 off

UnpulseSrvDone
	return

;----------------------------------------------------------
;End of Page1
;==========================================================

;==========================================================
;PAGE 2 -- The Interrupt Handlers Page
;----------------------------------------------------------
                ORG     h'1000'
;..........................................................

IntHandler			;External (RB0)
	btfss	INTCON,INTF	;Flagged?
	goto	RCIF_Check	;No, next check
	bcf	INTCON,INTF	;Clear flag
	goto	INTF_Handler	;Call handler				

RCIF_Check			;SCI received byte
	btfss	PIR1,RCIF	;Flagged?
	goto	SSPIF_Check	;No, next check
	bcf	PIR1,RCIF	;Clear flag
	goto	RCIF_Handler	;Call handler				

SSPIF_Check			;SPI tx/rcv occurred
	btfss	PIR1,SSPIF	;Flagged?
	goto	TMR2IF_Check	;No, next check
	bcf	PIR1,SSPIF	;Clear flag
	goto	SSPIF_Handler	;Call handler				

TMR2IF_Check			;Timer2 register overflow
	btfss	PIR1,TMR2IF	;Flagged?
	goto	IntHandlerDone	;No, done
	bcf	PIR1,TMR2IF	;Clear flag
	goto	TMR2IF_Handler	;Call handler

;----------------------------------------------------------
;External interrupt (RB0)

INTF_Handler
	bsf	RUNSTAT,BRK	;Set break flag
	goto	IntHandlerDone	;Done

;----------------------------------------------------------
;SCI received byte

RCIF_Handler
	btfsc	RCSTA,FERR	;Framing error?
	goto	RCIF_Error	;Yes, log it
	btfsc	RCSTA,OERR	;Overrun error?
	goto	RCIF_Overrun	;Yes, log it
	movfw	RCREG		;Get the char
	ISCall	RxPut		;Put it in RX FIFO
	goto	RCIF_Done	;Done

RCIF_Overrun
	bcf	RCSTA,CREN	;Reset SCI port
	bsf	RCSTA,CREN
	movlw	ERR_RCIFOR	;Log overflow error
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error
	goto	RCIF_Done

RCIF_Error
	movfw	RCREG		;Discard byte
	movlw	ERR_RCIFFE	;Log general error
	movwf	ERRSTAT		;Log to error status
	bsf	RUNSTAT,ERR	;Flag an error

RCIF_Done
	goto	IntHandlerDone	;Done

;----------------------------------------------------------
;SPI transmit/receive occurred

SSPIF_Handler

	;TO DO ...
	
	goto	IntHandlerDone	;Done

;----------------------------------------------------------
;Timer2 O/F Handler
; Timer2 overflow, occurs every 50 uSecs (.05 mS)

TMR2IF_Handler
;Countdown to end current servo pulse
	movfw	SERVOTIMER	;Get pulse timer
	btfsc	STATUS,Z	;Already zero?
	goto	TMR2IF_NextPulse	;Yes, skip
	
	decfsz	SERVOTIMER,F	;Decrement timer. Zero?
	goto	TMR2IF_NextPulse	;No, skip
	
	movfw	SERVOSELECT	;End current servo pulse
	ISCall	UnpulseServo	

;Countdown to next servo pulse
TMR2IF_NextPulse
	decfsz	SERVOTMRNEXT,F	;Decrement timer. Zero?
	goto	TMR2IF_MSec	;No, skip
	
;2.5ms passed, pulse next servo (if active)
	movlw	d'75'		;Reset Next timer
	movwf	SERVOTMRNEXT	;(75 * .05ms = 3.75ms)
				;(5 servos * 3.75ms = 18.75ms)
	incf	SERVOSELECT,F	;Select next servo
	movfw	SERVOSELECT
	sublw	d'4'		;In range (0-4)?
	btfss	STATUS,C	
	clrf	SERVOSELECT	;No, restart servo cycle

	movfw	SERVOSELECT	;Retrieve current servo delay	
	ISCall	GetServoDelay
	btfsc	STATUS,Z	;Zero?
	goto	TMR2IF_MSec	;Yes, inactive servo, skip	
	movwf	SERVOTIMER	;Set servo pulse timer
	
	movfw	SERVOSELECT	;Pulse current servo
	ISCall	PulseServo

;Check for Millisecond passed
TMR2IF_MSec
	decfsz	TMRHMSEC,F	;Decrement HMStimer; zero?
	goto	IntHandlerDone	;No, done
	movlw	d'20'		;Yes, reset HMStimer
	movwf	TMRHMSEC	;(1 millisecond passed)

;Update A/D, Servo Pulse, and Motor timers
TMR2IF_AD
	movfw	TMRAD		;A/D convsn. timer zero?
	btfss	STATUS,Z
	decf	TMRAD,F		;No, decrement it

;TMR2IF_Motor0
;	btfss	RUNSTAT,M0RUN	;Motor0 running?
;	goto	TMR2IF_Motor1	;No
;		
;	decfsz	TMRMOT0,F	;Yes, decrement timer.  Zero?
;	goto	TMR2IF_Motor1	;No, skip	
;	ISCall	Motor0Step	;Yes, step motor0
;	
;TMR2IF_Motor1
;	btfss	RUNSTAT,M1RUN	;Motor1 running?
;	goto	TMR2IF_MotorSet	;No
;
;	decfsz	TMRMOT1,F	;Yes, decrement timer.  Zero?
;	goto	TMR2IF_MotorSet	;No	
;	ISCall	Motor1Step	;Yes, step motor1
;
;Refresh motor port (whether it needs it or not)	
;TMR2IF_MotorSet
;	btfsc	RUNSTAT,M0RUN	;Motor 0 on?
;	goto	TMR2IF_MotorRef	;Yes, refresh port
;	btfsc	RUNSTAT,M1RUN	;No, Motor 1 on?
;	goto	TMR2IF_MotorRef	;Yes, refresh port
;	goto	TMR2IF_TSec	;No, skip refresh
;	
;TMR2IF_MotorRef
;	ISCall	MotorSet	;Refresh motor port

;Check for Tenth second passed
TMR2IF_TSec
	decfsz	TMRMSEC,F	;Decrement MSTimer; zero?
	goto	IntHandlerDone	;No, done
	movlw	d'100'		;Yes, reset MSTimer
	movwf	TMRMSEC		;(1/10th second passed)

;----------------------------------------------------------
;Interrupt handled, resume normal operation

IntHandlerDone
	bcf	PCLATH,3		;Page 0 (Main)
	bcf	PCLATH,4
	goto	Resume		;Resume from interrupt

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
