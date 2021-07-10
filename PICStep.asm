;==========================================================
; PICStep.asm
;----------------------------------------------------------
; Dedicated stepping motor controller, operating two stepper 
; motors from input from another (less dedicated) MCU.
;----------------------------------------------------------
; Current capabilities:	(See documentation)
;==========================================================

;----------------------------------------------------------
;Compile options

	title		"PICStep"
	list 		p=16f88
	
	;Include files
	include	 	"p16f88.inc"
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
	goto	MainInit		;Main program

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
	bcf	PCLATH,3		;Page 2 (Interrupts)
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
	goto	MainLoop		;Loop

;----------------------------------------------------------
