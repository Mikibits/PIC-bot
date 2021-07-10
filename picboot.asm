;
;  PicBoot.asm      Resident bootloader for the PIC 16F877
;
;  Derived from the original Microchip program (boot877.asm) by Karl Lunt.
;
;  Revised by Miki R. Marshall to shrink it down and simplify the 
;  operation, while creating a seamless interface with WinBot, my Windows 
;  user interface to the PIC via the SCI, which uses this bootloader to
;  download new user code. 
;

	list p=16f877, st=OFF, x=OFF, n=0
	errorlevel -302, -306
	#include <p16f877.inc>
	#include  "macros.inc"

	__CONFIG _BODEN_OFF & _CP_OFF & _PWRTE_ON & _WDT_OFF & _WRT_ENABLE_ON & _HS_OSC & _DEBUG_OFF & _CPD_OFF & _LVP_ON

;====================================================================
;Constants

TEST_PIN	equ     4       	; A4 = External bootload-on-reset (high)
BAUD_9600	equ     d'129'		; 9600 baud, assuming a 20 MHz chip
BAUD_DEFAULT	equ     BAUD_9600

;Variables in bank0

	CBLOCK  0x20
	AddressH:       1       ;flash program memory address high byte
	AddressL:       1       ;flash program memory address low byte
	NumWords:       1       ;number of words in line of hex file
	Checksum:       1       ;byte to hold checksum of incoming data
	Counter:        1       ;to count words being saved or programmed
	TestByte:       1       ;byte to show reset vector code received
	HexByte:        1       ;byte from 2 incoming ascii characters
	Temp:           1       ;temp byte for general use
	DataPointer:    1       ;pointer to data in buffer
	DataArray:      0x40    ;buffer for storing incoming data
	ENDC


; Reset vector ======================================================

	ORG     0x0000 

ResetVector:
	movlw   high Main
	movwf   PCLATH          ;set page bits for page3
	goto    Main            ;go to boot loader

; Bootload code start ===============================================

	ORG     0x1e00		;Use last part of page3 for PIC16F876/7
;	ORG     0x0e00		;Use last part of page1 for PIC16F873/4
;	ORG     0x0600		;Use last part of page0 for PIC16F870/1

StartOfBoot:
	movlw	high TrapError	;trap if execution runs into boot code
	movwf	PCLATH		;set correct page

; Trap accidental entry from user code

TrapError:
	goto	TrapError	;trap error and wait for reset

;First 4 lines of user code is relocated here, where jump-to-Main should be.

StartUserCode:
	clrf    PCLATH          ;set correct page for reset condition 
	nop                     ;relocated user code replaces this nop
	nop                     ;relocated user code replaces this nop
	nop                     ;relocated user code replaces this nop
	nop                     ;relocated user code replaces this nop
	movlw   high TrapError1 ;trap if no goto in user reset code
	movwf   PCLATH          ;set correct page
	
;Trap it here if goto is missing (must be in bank0)

TrapError1:
	goto    TrapError1      ;trap error and wait for reset

;Status word, indicates user program exists to be run

CodeStatus:
	DA      0x3fff          ;0 for valid code, 0x3fff for no code

;Test for external bootload signal (A4 high) on reset

Main:
	Bank0                   ;change to bank0 in case of soft reset
	btfsc   PORTA,TEST_PIN	;check pin for boot load               
	goto    Loader          ;if high then do bootload
	call    LoadStatusAddr  ;load address of CodeStatus word
	call    FlashRead       ;read data at CodeStatus location
	Bank2                   ;change from bank3 to bank2
	movf    EEDATA,F        ;set Z flag if data is zero
	Bank0                   ;change from bank2 to bank0
	btfss   STATUS,Z        ;test Z flag

TrapError2:
	goto    TrapError2      ;if not zero then is no valid code
	goto    StartUserCode   ;if zero then run user code

; Start (internal) ==================================================

Loader:
        Bank0			;entry from vector at end of ROM
        clrf    TestByte	;indicate no reset vector code yet
        call    LoadStatusAddr	;load address of CodeStatus word
        movlw   0x3f		;load data to indicate no program
        movwf   EEDATH
        movlw   0xff		;load data to indicate no program
        movwf   EEDATA
        call    FlashWrite	;write new CodeStatus word
        call    SerialSetup	;set up serial port

; Send ready-to-bootload signal

ReadyToBoot:
	movlw   'P'
        call    SendChar
        movlw   'B'
        call    SendChar
        call	SendCRLF

; Immediately drop into data upload mode

LoadNewRec:
        call    ReceiveChar	; get new byte from serial port
        xorlw   0x0a            ; linefeed?
        btfsc   STATUS,Z
        goto    LoadNewRec      ; yes, ignore it
        xorlw   0x0a            ; restore char
        xorlw   0dh             ; not :, is it CR?
        btfsc   STATUS,Z
        goto    LoadNewRec      ; got CR, just start over
        xorlw   0dh             ; restore the char
        xorlw   ':'             ; check if ':' received
        btfsc   STATUS,Z
        goto    GetRec          ; got :, start processing record
        xorlw   ':'             ; restore the char
        xorlw   h'1B'           ; <Esc> received?
        btfsc   STATUS,Z
        goto    Loader		; yes, restart loader
        xorlw	h'1B'		; restore the char
        xorlw	'R'		; Run command received?
        btfsc   STATUS,Z
        goto    DoRunCmd	; yes, do run command
        xorlw   'R'             ; restore the char
        xorlw   'r'             ; Run command received?
        btfsc   STATUS,Z
        goto    DoRunCmd	; yes, do run command
	goto	ErrorMessage0	; Error: Unexpected char

;  Load a HEX record (everything after ':')

GetRec:
        clrf    Checksum        ; start with checksum zero
        call    GetHexByte      ; get number of program data bytes in line
        andlw   0x1F            ; limit number in case of error in file
        movwf   NumWords
        bcf     STATUS,C
        rrf     NumWords,F      ; divide by 2 to get number of words
        call    GetHexByte      ; get upper half of program start address
        movwf   AddressH

        call    GetHexByte      ; get lower half of program start address
        movwf   AddressL

        bcf     STATUS,C
        rrf     AddressH,F      ;divide address by 2 to get word address
        rrf     AddressL,F

        call    GetHexByte      ;get record type
        xorlw   0x01
        btfsc   STATUS,Z        ;check if end of file record (0x01)
        goto    FileDone        ;if end of file then all done

        movf    HexByte,W
        xorlw   0x00
        btfss   STATUS,Z        ;check if regular line record (0x00)
        goto    LineDone        ;if not then ignore line

        movlw   0xe0
        addwf   AddressH,W      ;check if address >= 0x2000 (was <)
        btfsc   STATUS,C        ;which is ID locations and config bits
        goto    LineDone        ;if so then ignore line

;  Get data bytes and checksum from hex file

        movlw   DataArray
        movwf   FSR             ;set pointer to start of array
        movf    NumWords,W
        movwf   Counter         ;set counter to number of words

GetData:
        call    GetHexByte      ;get low data byte
        movwf   INDF            ;save in array
        incf    FSR,F           ;point to high byte

        call    GetHexByte      ;get high data byte
        movwf   INDF            ;save in array
        incf    FSR,F           ;point to next low byte

        decfsz  Counter,F
        goto    GetData

        call    GetHexByte      ;get checksum
        movf    Checksum,W      ;check if checksum correct
        btfss   STATUS,Z
        goto    ErrorMessage1

;  Get saved data one word at a time to program into flash 

        movlw   DataArray
        movwf   FSR             ;point to start of array
        movf    NumWords,W
        movwf   Counter         ;set counter to half number of bytes

;  Check if address is in reset code area

CheckAddress:
        movf    AddressH,W      ;checking for boot location code
        btfss   STATUS,Z        ;test if AddressH is zero 
        goto    CheckAddress1   ;if not go check if reset code received

        movlw   0xfc    
        addwf   AddressL,W      ;add 0xfc (-4) to address
        btfsc   STATUS,C        ;no carry means address < 4
        goto    CheckAddress1   ;if not go check if reset code received

        bsf     TestByte,0      ;show that reset vector code received
        movf    AddressL,W      ;relocate addresses 0-3 to new location
        addlw   low (StartUserCode + 1) ;add low address to new location
        Bank2                   ;change from bank0 to bank2
        movwf   EEADR           ;load new low address
        movlw   high (StartUserCode + 1) ;get new location high address
        movwf   EEADRH          ;load high address
        goto    LoadData        ;go get data byte and program into flash

;  Check if reset code has been received
;  Check if address is too high and conflicts with boot loader

CheckAddress1:
        btfss   TestByte,0      ;check if reset vector code received first
        goto    ErrorMessage2   ;if not then error

        movlw   high StartOfBoot ;get high byte of address
        subwf   AddressH,W
        btfss   STATUS,C        ;test if less than boot code address 
        goto    LoadAddress     ;yes so continue with write
        btfss   STATUS,Z        ;test if equal to boot code address 
        goto    ErrorMessage3   ;no so error in high byte of address

        movlw   low StartOfBoot ;get low byte of address
        subwf   AddressL,W
        btfsc   STATUS,C        ;test if less than boot code address 
        goto    ErrorMessage3   ;no so error in address

;  Load address and data;  write data into flash

LoadAddress:
        movf    AddressH,W      ;get high address
        Bank2                   ;change from bank0 to bank2
        movwf   EEADRH          ;load high address
        Bank0                   ;change from bank2 to bank0
        movf    AddressL,W      ;get low address
        Bank2                   ;change from bank0 to bank2
        movwf   EEADR           ;load low address

LoadData:
        movf    INDF,W          ;get low byte from array
        movwf   EEDATA          ;load low byte
        incf    FSR,F           ;point to high data byte
        movf    INDF,W          ;get high byte from array
        movwf   EEDATH          ;load high byte
        incf    FSR,F           ;point to next low data byte

        call    FlashWrite      ;write data to program memory

        Bank0                   ;change from bank3 to bank0
        incfsz  AddressL,F      ;increment low address byte
        goto    CheckLineDone   ;check for rollover
        incf    AddressH,F      ;if so then increment high address byte

CheckLineDone:
        decfsz  Counter,F       ;check if all words have been programmed
        goto    CheckAddress    ;if not then go program next word

;  Done loading this line, get ready for next one

LineDone:
        call    ReceiveChar 	; eat ignored records, if any
        xorlw   0x0d            ; hit the CR yet?
        btfss   STATUS,Z
        goto	LineDone	; no, continue looking
        goto    LoadNewRec      ; go get next line hex file

;  Loaded entire file successfully, display status

FileDone:

File1:
; EOF record hit--ignore the rest

        call    ReceiveChar     ; need to eat checksum for file record
        xorlw   0x0d            ; hit the CR yet?
        btfss   STATUS,Z
        goto    File1           ; loop until CR

File2:
        call    LoadStatusAddr  ;load address of CodeStatus word
        clrf    EEDATH          ;load data to indicate program exists
        clrf    EEDATA          ;load data to indicate program exists
        call    FlashWrite

        call    SendCRLF
        movlw   'O'		;Send success status
        call    SendChar
        movlw   'K'
        call    SendChar
        call    SendCRLF
                                ;Fall through to Run program
;;;     goto    ReadyToBoot

;  RUN selected, jump to user program if code exists, otherwise send error

DoRunCmd:
        call    LoadStatusAddr          ; load address of CodeStatus word
        call    FlashRead               ; read data at CodeStatus location
        Bank2                           ; change from bank3 to bank2
        movf    EEDATA,F                ; set Z flag if data is zero
        Bank0                           ; change from bank2 to bank0
        btfss   STATUS,Z                ; skip if code status is good
        goto	ErrorMessage4		; Error: No code to run
        goto    StartUserCode           ; run the user program

;  Display an error

ErrorMessage0:
        call    SendCRLF	;Let sender know we're not loading
        movlw   'E'		;E0:  Unexpected char error
        call    SendChar
        movlw   '0'
        call    SendChar
        call    SendCRLF
        goto    ReadyToBoot

ErrorMessage1:
        call    SendCRLF	;Let sender know we're not loading
        movlw   'E'		;E1:  Checksum error
        call    SendChar
        movlw   '1'
        call    SendChar
        call    SendCRLF
        goto    ReadyToBoot  

ErrorMessage2:
        call    SendCRLF	;Let sender know we're not loading
        movlw   'E'		;E2:  Reset vector code not first
        call    SendChar
        movlw   '2'
        call    SendChar
        call    SendCRLF
        goto    ReadyToBoot  

ErrorMessage3:
        call    SendCRLF	;Let sender know we're not loading
        movlw   'E'		;E3:  Boot code overrun
        call    SendChar
        movlw   '3'
        call    SendChar
        call    SendCRLF
        goto    ReadyToBoot  

ErrorMessage4:
        call    SendCRLF	;Let sender know we're not loading
        movlw   'E'		;E4:  No code to run
        call    SendChar
        movlw   '4'
        call    SendChar
        call    SendCRLF
        goto    ReadyToBoot  

; Subroutines =======================================================

;  Send a CR/LF sequence

SendCRLF:
        movlw   0x0d		; CR
        call    SendChar
        movlw   0x0a		; LF
        call    SendChar
        return

;  Load address of CodeStatus word to flash addr regs (returns in bank2)

LoadStatusAddr:
        Bank2                   ;change from bank0 to bank2
        movlw   high CodeStatus ;load high addr of CodeStatus location
        movwf   EEADRH
        movlw   low CodeStatus  ;load low addr of CodeStatus location
        movwf   EEADR
        return

;  Get two ASCII digits, convert to single byte (in W); add to checksum
;  (returns in bank0)

GetHexByte:
        call    ReceiveChar	;get new byte from serial port
        addlw   0xbf            ;add -'A' to Ascii high byte
        btfss   STATUS,C        ;check if positive
        addlw   0x07            ;if not, add 17 ('0' to '9')
        addlw   0x0a            ;else add 10 ('A' to 'F') 
        movwf   HexByte         ;save nibble
        swapf   HexByte,F       ;move nibble to high position

        call    ReceiveChar	;get new byte from serial port
        addlw   0xbf            ;add -'A' to Ascii low byte
        btfss   STATUS,C        ;check if positive
        addlw   0x07            ;if not, add 17 ('0' to '9')
        addlw   0x0a            ;else add 10 ('A' to 'F') 
        iorwf   HexByte,F       ;add low nibble to high nibble
        movf    HexByte,W       ;put result in W reg
        addwf   Checksum,F      ;add to cumulative checksum
        return

;  Initialize the SCI (USART) for transmit and receive at the default
;  baud rate.  (returns in bank0)

SerialSetup:
        Bank1
        movlw   BAUD_DEFAULT    ;set baud rate 
        movwf   SPBRG
        bsf     TXSTA,BRGH      ;baud rate high speed option
        bsf     TXSTA,TXEN      ;enable transmission
        Bank0                   ;change from bank1 to bank0
        bsf     RCSTA,CREN      ;enable reception
        bsf     RCSTA,SPEN      ;enable serial port
        return

;  Read a byte from SCI, return in W (returns in bank0)

ReceiveChar:
        Bank0                   ;change from unknown bank to bank0
        btfss   PIR1,RCIF       ;check if data received
        goto    $-1             ;wait until new data
        movf    RCREG,W         ;get received data into W
        ;; Fall thru -- echo everything received --
	
;  Sends a byte in W register via SCI (returns in bank0)

SendChar:
        Bank0                   ;change from unknown bank to bank0
        btfss   PIR1,TXIF       ;check that buffer is empty
        goto    $-1
        movwf   TXREG           ;transmit byte
        return

;  Write to a location in the flash program memory, given the address 
;  in EEADRH/EEADR and the data in EEDATH/EEDATA (returns in bank3)

FlashWrite:
        Bank3                   ;change from bank2 to bank3
        movlw   0x84            ;enable writes to program flash
        movwf   EECON1

        movlw   0x55            ;do timed access writes
        movwf   EECON2
        movlw   0xaa
        movwf   EECON2
        bsf     EECON1,WR       ;begin writing to flash

        nop                     ;processor halts here while writing
        nop
        return

;  Read from a location in the flash program memory, given the address
;  in EEADRH/EEADR, data returned in EEDATH/EEDATA (returns in bank3)

FlashRead:
	Bank2                   ; get it right
	movlw   0x1f            ; keep address within range
	andwf   EEADRH,F

	Bank3                   ; change from bank2 to bank3
	movlw   0x80            ; enable reads from program flash
	movwf   EECON1
	bsf     EECON1,RD       ; read from flash
	nop                     ; processor waits while reading
	nop
	return

;  The following instructions act as vectors to permit other programs
;  access to routines within the monitor.  Programs should execute
;  a CALL to one of these addresses, rather than a CALL directly to
;  the target routine, as the target routine might move in future
;  versions of this program!
;
;  Pay close attention to the banking when these routines exit, as
;  they often change the current register bank.
;
;  Note that entry to the Loader at $xxff should be via a GOTO, not
;  a CALL.
;

        org     StartOfBoot+0x1f8

        return                          ; $1ff8
        return                          ; $1ff9
        return                          ; $1ffa
        return                          ; $1ffb
        goto    SerialSetup             ; $1ffc
        goto    FlashRead               ; $1ffd
        goto    FlashWrite              ; $1ffe
        goto    Loader                  ; $1fff
        
;--------------------------------------------------------------------

        END
