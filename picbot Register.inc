;==========================================================
; Register.inc
;----------------------------------------------------------
; Register definitions (variables and constants) for 
; Bot1 (PicBot1) project.
;==========================================================

;==========================================================
;Register variables

;----------------------------------------------------------
;Shared-Bank (70h - 7Fh)

;Save registers (for interrupts)
SAVINTW		equ     h'70'	;W reg. for Int
SAVINTSTAT	equ     h'71'	;STATUS reg. for Int
SAVINTFSR	equ     h'72'	;FSR reg. for Int
SAVINTPCL	equ     h'73'	;PCLATH reg. for Int
SAVSELECT	equ	h'74'	;Lookup SELECT for Int

;Status registers
RUNSTAT		equ	h'75'	;Runtime status
SCISTAT		equ	h'76'	;SCI FIFO statuses
SPISTAT		equ	h'77'	;SPI FIFO statuses
ERRSTAT		equ	h'78'	;Error status

;SCI data variables
RXCHAR		equ	h'79'	;RX character hold
RXCMDCNT	equ	h'7A'	;RX command count
RXTEMP		equ	h'7B'	;RxPut hold
TXTEMP		equ	h'7C'	;TxPut hold

;SPI data variables
SPICHAR		equ	h'7D'	;SPI character hold

;----------------------------------------------------------
;Bank0 (20h - 6Fh)

;Timer 2 registers
TMRHMSEC	equ	h'20'	;Hundredths of mSecs (10 uSecs)
TMRMSEC		equ	h'21'	;Milliseconds (100 TMRHMSEC's)
TMRTSEC		equ	h'22'	;Tenth seconds (100 TMRMSEC's)

;Timer 2 Countdown Timers by function
TMRAD		equ	h'23'	;A/D conversion timer
TMRMOT0		equ	h'25'	;Motor 0 step delay
TMRMOT1		equ	h'26'	;Motor 1 step delay
SERVOTIMER	equ	h'27'	;Servo pulse duration timer
SERVOTMRNEXT	equ	h'28'	;Countdown to next servo pulse

;Status Registers
M0SPEED		equ	h'31'	;Motor 0 speed
M1SPEED		equ	h'32'	;Motor 1 speed
M0PHASE		equ	h'33'	;Motor 0, current phase
M1PHASE		equ	h'34'	;Motor 1, current phase
MTEMP		equ	h'35'	;Motor phase bit hold
SERVOSELECT	equ	h'36'	;Servo selector (0-4)
SERVOSTAT	equ	h'37'	;Servo active bits
SERVODELAY0	equ	h'38'	;Servo delay settings (0-4)
SERVODELAY1	equ	h'39'
SERVODELAY2	equ	h'3A'
SERVODELAY3	equ	h'3B'
SERVODELAY4	equ	h'3C'

;Port/pin address variables
PORTPIN		equ	h'40'	;Register char
PORT		equ	h'41'	;Register address
PIN		equ	h'42'	;Pin/bit number
PINMODE		equ	h'43'	;Pin mode status

;A/D registers
ADCH		equ	h'48'	;A/D channel
ADWAIT		equ	h'49'	;A/D timer

;Work registers
CNTR		equ	h'60'	;Miscellaneous counter
SELECT		equ	h'61'	;Table selection (offset)
TEMP		equ	h'62'	;General use (beware)
SERVOTEMP	equ	h'63'	;Servo 
PINTEMP		equ	h'64'	;Pin Status
HEXBYTE		equ	h'65'	;Hex byte
WTEMP		equ	h'66'	;Hold W for paging macros

;----------------------------------------------------------
;Bank1 (A0h - EFh)

;SCI FIFO buffers (46 bytes)
RXFIFO		equ	h'A0'	;SCI Receive FIFO
RXFIFOMAX	equ	h'CD'	;RXFIFO extent

;SCI FIFO pointers
RXPUTPTR	equ	h'CE'	;RX insertion pointer
RXGETPTR	equ	h'CF'	;RX extraction pointer

;SPI FIFO buffers (46 bytes)
SPIFIFO		equ	h'D0'	;SPI Receive FIFO
SPIFIFOMAX	equ	h'FD'	;SPIFIFO extent

;SPI FIFO pointers
SPIPUTPTR	equ	h'FE'	;SPI insertion pointer
SPIGETPTR	equ	h'FF'	;SPI extraction pointer

;----------------------------------------------------------
;Bank2 (120h - 16Fh)

;----------------------------------------------------------
;Bank3 (1A0h - 1EFh)

;End of Register Variables
;==========================================================

;==========================================================
;Constants

;SCISTAT status bits
RXFULL		equ	h'0007'	;Receive FIFO full
RXEMPTY		equ	h'0006'	;Receive FIFO empty
HOSTUP		equ	h'0005'	;Host is online

;RUNSTAT status bits
ERR		equ	h'0007'	;An error occurred
BRK		equ	h'0006'	;User break (ext. int.)
RXCMD		equ	h'0005'	;Received a command
RXERR		equ	h'0004'	;Received an error
M1FWD		equ	h'0003'	;Motor 1: 0=Rev, 1=Fwd
M1RUN		equ	h'0002'	;Motor 1: 0=Stop, 1=Go
M0FWD		equ	h'0001'	;Motor 0: 0=Rev, 1=Fwd
M0RUN		equ	h'0000'	;Motor 0: 0=Stop, 1=Go

;PINMODE status bits
OUT		equ	h'0007'	;Pin is an output
AD		equ	h'0006'	;Pin is A/D input
SRV		equ	h'0005' ;Pin is servo output

;Error codes	(ascii, 0-7 reserved for bootloader)
ERR_RXPUTOF	equ	'8'	;RxPut overflow
ERR_RXGETUF	equ	'9'	;RxGet underflow
ERR_RCIFOR	equ	'A'	;SCI: RC overrun
ERR_RCIFFE	equ	'B'	;SCI: RC frame error
ERR_BADCMD	equ	'C'	;Bad command received
ERR_BADSRVPOS	equ	'D'	;Bad servo position
ERR_BADPORT	equ	'E'	;Bad port address
ERR_BADMOTNO	equ	'G'	;Bad motor number
ERR_BADMOTDIR	equ	'H'	;Bad motor direction
ERR_BADMOTSPD	equ	'I'	;Bad motor speed


;ASCII characters
CR		equ	h'0D'	;<CR>
LF		equ	h'0A'	;<LF>

;End of Constants
;==========================================================
