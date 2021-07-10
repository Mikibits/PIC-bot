;==========================================================
; Register.inc
;----------------------------------------------------------
; Register definitions (variables and constants) for 
; Bot1 (PicBot1) project.
;==========================================================

;;; NOTE: Example code culled from PicBot Prototype--See
;;;       that code for supporting information regarding
;;;	      the setup of interrupts and timers to make the
;;;		  following code work.

;==========================================================
;Register variables

;----------------------------------------------------------
;Shared-Bank (70h - 7Fh)

;;None

;----------------------------------------------------------
;Bank0 (20h - 6Fh)

;Timer 2 Countdown Timers by function
TMRMOT0		equ	h'25'	;Motor 0 step delay
TMRMOT1		equ	h'26'	;Motor 1 step delay

;Status Registers
MOTSTAT		equ	h'31'	;Motor (direction/speed)
MOTSTEP0	equ	h'32'	;Motor 0, current phase#
MOTSTEP1	equ	h'33'	;Motor 1, current phase#

;Work registers
MOTTEMP		equ	h'67'	;Motor stuff


;----------------------------------------------------------
;Bank2 (120h - 16Fh)

;----------------------------------------------------------
;Bank3 (1A0h - 1EFh)

;End of Register Variables
;==========================================================

;==========================================================
;Constants

;MOTTEMP status bit
FWD1		equ	h'0007'	;Motor1: 0=Rev, 1=Fwd
FWD0		equ	h'0004'	;Motor0: 0=Rev, 1=Fwd

;Error codes
ERR_BADMOTNO	equ	h'0B'	;Bad motor number
ERR_BADMOTMD	equ	h'0C'	;Bad motor mode

;Stepping Motor phase values
MOTPHASEA	equ	b'00001010'
MOTPHASEB	equ	b'00000110'
MOTPHASEC	equ	b'00000101'
MOTPHASED	equ	b'00001001'

;End of Constants
;==========================================================
