;==========================================================
; macros.inc
;----------------------------------------------------------
; Macro definitions for the Bot1 project.
;==========================================================

;==========================================================
;Macros

;----------------------------------------------------------
;Data Bank selectors

Bank0	macro
	bcf     STATUS,RP0      ;Bank 0
	bcf     STATUS,RP1
	bcf     STATUS,IRP
	endm

Bank1	macro
	bsf     STATUS,RP0      ;Bank 1
	bcf     STATUS,RP1
	bcf     STATUS,IRP
	endm

Bank2	macro
	bcf     STATUS,RP0      ;Bank 2
	bsf     STATUS,RP1
	bsf     STATUS,IRP
	endm

Bank3	macro
	bsf     STATUS,RP0      ;Bank 3
	bsf     STATUS,RP1
	bsf     STATUS,IRP
	endm

;----------------------------------------------------------
;Interrupt & subroutine call routines that handle paging

;Call an Interrupt Handler(Page 2) from Main(0)
ICall	macro   label
	bcf     PCLATH,3        ;Page 2 (Interrupts)
	bsf     PCLATH,4
	call    label           ;Call interrupt handler
	bcf     PCLATH,3        ;Page 0 (Main)
	bcf     PCLATH,4
	endm

;Call a Subroutine(1) from Main(0)
SCall	macro   label
	movwf	WTEMP		;Save W
	movlw	high (label)	;Get all bits for PCLATH*
	movwf	PCLATH		;Set page
	movfw	WTEMP		;Restore W
	call    label           ;Call subroutine
	bcf     PCLATH,3        ;Page 0 (Main)
	bcf     PCLATH,4
	endm

;Call a Subroutine(1) from an Interrupt(2)
ISCall	macro   label
	movwf	WTEMP		;Save W
	movlw	high (label)	;Get all bits for PCLATH*
	movwf	PCLATH		;Set page
	movfw	WTEMP		;Restore W
	call    label           ;Call subroutine
	bcf     PCLATH,3        ;Page 2 (Interrupts)
	bsf     PCLATH,4
	endm

;  * Note:  Setting only bits 3 & 4 in PCLATH don't work when
;           calling lookup tables, as bit 8 is lost whenever
;           PCLATH is set, which causes problems when lookup
;           tables use "addwf PCL,F" above the first 256K of
;           a page.  The above code fixes this.

;End of Macros
;==========================================================
