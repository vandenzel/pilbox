// pilbox/main.cpp

/*
 * Original version written in PIC16 assembly by Jean-Francois Garnier
 * 
 * Porting attempt to Arduino Pro Micro by Alex van Denzel
 */

/* Version history (by Jean-Francois Garnier)
 * PILbox1.asm
 * PIL-Box version 1 - HPIL/Serial Translator based on PIC uP
 * History:
 * 28/10/07 prototype code for feasability
 * 13/01/08 test with  PIC internal comparators
 * 08/03/08 translator implementation
 * 25/10/08 some cleanup and debugging
 * 28/11/08 version 1.0: more cleanup, documentation
 * 08/12/08 version 1.1: fixed problem with HP-71B RESTORE IO sequence
 * 29/06/09 version 1.2: added controller mode, enable bit, SRQ support on IDY frames,
 *                       fast speed, watchdog, code protected
 * 21/07/09 version 1.21 fixed pb with HP-71B (again) - reported by Patrice T.
 *                       improved fast mode (no ack needed in fast mode)
 * 31/08/09 version 1.3  renamed to 1.3 for new PILBox batch
 * 18/10/09 version 1.31 attempt to fix issue on some HP71 HPIL modules rev 1A (Marcus v.C.)
 * 19/10/09 version 1.32 
 * 21/10/09 version 1.33 optional fix for module 1A
 * 27/10/09 version 1.4  renamed to 1.4 after tests by Marcus
 *
 * 28/03/14 version 1.41 improve USB transfer rate by using FTDI ctrl lines
 * 04/04/14 version 1.42 improve frame decoding
 * 17/04/14 version 1.43 more improvement of frame decoding
 * 23/04/14 version 1.5  renamed to 1.5 after tests
 * 27/09/15 version 1.51 COFI mode (TRIDY flag), minor speed optimization
 * 05/10/15 version 1.6  renamed to 1.6 after tests
 * to do: add mailbox mode
 */

#include <Arduino.h>

/* configuration bits as set in the PIC16 micro:
 *      HS osc
 *      Watchdog on
 *      Powerup timer on
 *      Code protect on
 *      Brownout enabled
 *      Master clear disabled
 *      Low voltage prog disabled
 *      Data EE protect disabled 
 */

// 20-6f: bank of 80 bytes
#define BUF     0x20            // buffer for IL frame samples (20-5f)
#define DECFLG  0x60            // decoding flags
#define CPTBIT  0x62            // bit counter
#define CPTSAM  0x63            // sample counter
#define FRREGH  0x64            // input/output frame shift registers
#define FRREGL  0x65
#define SAVEW   0x6f            // save W during IT
// 70-7f: commun area 
#define ILFLG   0x70            // IL flags 
#define FRREC   0               // frame received
#define FRAV    1               // frame available  
#define FCMD    2               // command frame received
#define SSRQ    3               // set SRQ bit in IDY
#define TRIDY   4               // translate IDY
#define MBOX    5               // mailbox mode (not yet used)
#define TRENA   6               // translator enable 
#define CTRL    7               // controller on (in translator mode)
#define CPTB    0x71            // general purpose counter
#define FRAMEH  0x72            // IL 11-bit frame in 72-73
#define FRAMEL  0x73
#define SBYTE   0x74            // byte from/to serial
// variables for translator mode:
#define HBYT    0x75            // high/low frame bytes from/to serial
#define LBYT    0x76
#define LASTH   0x77            // last serial bytes
#define LASTL   0x78
// ----------
#define LASTC   0x79            // last command frame

#define FIX1A   7               // enable HP71B HPIL 1A fix (closed) - RB7
#define LSPEED  6               // serial speed: 9.6k (open) else 115k (closed) - RB6
#define RTSO    5               // RTS output control line (RS232) - RB5
#define CTSI    4               // CTS input  control line (RS232) - RB4 (not connected)
#define LEDOFF  4               // LED command - RA4




//         org     0
//         goto    Start

//         org     4
// Intr
// ; interrupt routine to sample the HPIL frame
//         movwf   SAVEW       ; save W
// ; fast entry point for fix
// Intr1        
//      variable i
// i=0
//      while i<D'64'   ; 128 samples stored in 64 registers
//         movf    CMCON,w     ; ABxx11xx
//         movwf   BUF+i
//         swapf   CMCON,w     ; 11xxCDxx
//         andwf   BUF+i,f     ; ABxxCDxx
// i+=1
//      endw
//         bcf     PIR1, CMIF   ; reset IT cause
//         bsf     ILFLG,FRREC
//         movf    SAVEW,w      ; restore W and set STATUS,Z accordingly
//         retfie

// ; -----------------------------------------------
//         dt "J-F GARNIER, 2009-2015"
// ; -----------------------------------------------

// Start
//         clrwdt
//         movlw   B'00000000'     ; port outputs
//         movwf   PORTA
//         movwf   PORTB
//         bsf     STATUS, RP0     ; *****************
//         movlw   B'11101111'     ;make port a4 output
//         movwf   TRISA           ;       /
//         movlw   B'11010010'     ;make port b0,2,3,5 outputs
//         movwf   TRISB           ;       /
//         bcf     OPTION_REG,7    ; weak pull-up on port B
//         call    Dly100us        ; delay 100us

// ; -- setup of the UART
// ; BRG programmed at high speed to 9.6k/19.2k/57.6k/115.2k  
// ;                 SPBRG code=     129 / 64  / 21  / 10    
//         bcf     STATUS, RP0  ; *************************
//         movlw   D'129'  
//         btfss   PORTB,LSPEED
//         movlw   D'10'
//         bsf     STATUS, RP0  ; *************************
//         movwf   SPBRG
// ; setup Rx and Tx
//         movlw   0x24         ; Tx enable, async high speed
//         movwf   TXSTA        
//         bcf     STATUS, RP0  ; *************************
//         movlw   0x90         ; Serial enable, continous receive
//         movwf   RCSTA        

// ; -- setup of the comparators
//         movlw   0x1c         ;  mode 100, C1INV
//         movwf   CMCON
//         nop
//         movf    CMCON,w       ; read

// ; enable INT on comparator change
//         bsf     STATUS, RP0  ; *************************
//         movlw   0x40            ; CMIE
//         movwf   PIE1
//         bcf     STATUS, RP0  ; *************************
//         movlw   0x40            ; PEIE
//         movwf   INTCON

// ; init variables        
//         clrf    ILFLG
//         clrf    LASTH
//         clrf    LASTL

// ; make blink the LED
//         call    blinkLED

//         bsf     INTCON, GIE ; enable intr


void loop(void) {
    clrwdt();

    if (ILFLG & (1 << FRREC)) {
        insamp();
    }

    if (ILFLG & (1<< FRAV)) {
        _goto(inIL);
    }

    if (Serial.available()) {
        _goto(inserial)
    }

}
        
void insamp() {
    cli();
    led_on();
    decframe();
    sei();
    led_off();
}


        
// ; -------------------------------        
// ; IL frame available.
// ; -------------------------------
// inIL    bcf     INTCON, GIE   ; disable intr
//         bcf     PORTA,LEDOFF  ; LED on
//         bcf     ILFLG,FRAV
//         ; test if translator enabled
//         btfss   ILFLG,TRENA
//         goto    TR11       ; no, just retransmit
//         ; according to frame type:
//         btfss   FRAMEH,2   ; control frame?
//         goto    TR12       ;   no (DOE), translates it
//         btfsc   FRAMEH,1   ; CMD or RDY?
//         goto    TR09       ;   no (IDY)
//         btfss   FRAMEH,0   ; RDY?
//         goto    TR10       ;   no (CMD)
//         movf    FRAMEL,w   ; 
//         btfss   STATUS,Z   ; RFC?
//         goto    TR12       ;   no (RDY), translates it
        
//         ; RFC frames
//         btfss   ILFLG,FCMD
//         goto    TR08
//         bcf     ILFLG,FCMD
//         ; translate previously received command
//         movf    LASTC,w
//         movwf   FRAMEL
//         movlw   0x04
//         movwf   FRAMEH
//         goto    TR12
// TR08    btfss   ILFLG,CTRL
//         goto    TR11      ; non-controller mode, re-transmit
//         goto    TR12      ; controller, translate

// TR09    ; IDY frames 
//         btfsc   ILFLG,CTRL
//         goto    TR12      ; controller mode, translate back
//         ; non-ctrl, 
//         btfsc   ILFLG,TRIDY ; IDY translate enabled? (new 1.51)
//         goto    TR12       ; enabled, translate
//         ; update IDY frame and retransmit
//         btfsc   ILFLG,SSRQ
//         bsf     FRAMEH,0   ; set bit SRQ in IDY frame
//         goto    TR11
        
// TR10    ; CMD frames
//         movf    FRAMEL,w
//         movwf   LASTC
//         bsf     ILFLG,FCMD
//         btfss   ILFLG,CTRL
//         goto    TR11    ; non-ctrl: retransmit
//         ; ctrl: send RFC
//         movlw   0x05
//         movwf   FRAMEH
//         clrf    FRAMEL
// TR11    
//         goto    sendIL      ; returns to Main
        

// TR12    ; frame to be translated 
//         ; build low and high parts
//         movf    FRAMEL,w
//         movwf   LBYT
//         movf    FRAMEH,w
//         movwf   HBYT
//         ; test for last format used for low byte:
//         btfss   LASTL,7
//         goto    TR13
//         ; 1ddd dddd          
//         rlf     LBYT,w
//         rlf     HBYT    ; 4 higher bits
//         rlf     HBYT,w
//         andlw   0x1e
//         iorlw   0x20    ; format = 001c ccd0
//         movwf   HBYT
//         bsf     LBYT,7  ; format = 1ddd dddd
//         goto    TR14
        
// TR13    ; 01dd dddd
//         rlf     LBYT
//         rlf     HBYT
//         rlf     LBYT,w
//         rlf     HBYT,w    ; 5 higher bits
//         andlw   0x1f
//         iorlw   0x20    ; format = 001c ccdd
//         movwf   HBYT
//         rrf     LBYT
//         bcf     LBYT,7
//         bsf     LBYT,6  ; format = 01dd dddd
        
// TR14    ; if high part different than last one, send it
//         movf    LASTH,w
//         subwf   HBYT,w
//         btfsc   STATUS,Z
//         goto    TR15      ; else send low part only
//         movf    HBYT,w
//         movwf   LASTH
//         call    sndserial
//         call    Dly100us   ; moved here for speed optimization (1.51)
//         btfsc   PORTB,LSPEED  ; send also low part in high speed mode
//         goto    TR16
// TR15    ; send low part
//         movf    LBYT,w
//         movwf   LASTL
//         call    sndserial
//         call    Dly100us    ; moved here for speed optimization (1.51)  
// TR16    ; --- ver 1.41 change:
//         btfsc   PORTB,LSPEED
//         goto    TR17
//         ; in high speed, pulse RTSO (FTDI CTS) to force transmission
//         bsf     PORTB,RTSO
//         nop
//         nop
//         nop
//         bcf     PORTB,RTSO
//         ; --- end of change
// TR17    bsf     INTCON, GIE    ; enable intr
//         bsf     PORTA,LEDOFF   ; LED off
//         goto    Main    
        
        
       

// ; -------------------------------        
// ; serial byte received.
// ; -------------------------------        
// inserial
//         bcf     INTCON, GIE    ; desable intr
//         bcf     PORTA,LEDOFF   ; LED on
//         movf    RCREG,W
//         movwf   SBYTE
//         btfss   RCSTA,OERR
//         goto    inSER1
// ; in case of USART error, reset it
//         bcf     RCSTA,CREN
//         nop
//         bsf     RCSTA,CREN        
// inSER1         
//         ; check for high byte ack
//         btfss   PORTB,LSPEED
//         goto    TR22   ; ignore ack in high speed
//         movf    SBYTE,w
//         sublw   0x0D  ; CR?
//         btfsc   STATUS,Z
//         goto    TR20
//         movf    SBYTE,w
//         sublw   0x0A  ; LF?
//         btfss   STATUS,Z
//         goto    TR22
// TR20    ; send low byte
//         movf    LBYT,w
//         btfsc   STATUS,Z   ; already sent?
//         goto    TR21       ; yes
//         movwf   LASTL
//         call    sndserial
//         clrf    LBYT       ; sent!
// TR21    goto    TR40

// TR22  ; test for low/high byte
//         btfsc   SBYTE,7
//         goto    TR23   ; low byte
//         btfsc   SBYTE,6
//         goto    TR23   ; low byte
//         btfss   SBYTE,5
//         goto    TR40   ; invalid
//         ; high byte, store it
//         movf    SBYTE,w
//         movwf   HBYT
//         goto    TR40

// TR23    ; else low byte
//         movf    SBYTE,w
//         movwf   LASTL
//         ; was a high byte received?
//         movf    HBYT,w
//         btfsc   STATUS,Z
//         goto    TR28  ; no
//         ; yes, use it
//         movwf   LASTH
//         clrf    HBYT
// TR28    ; assemble frame
//         movf    LASTL,w
//         movwf   FRAMEL
//         movf    LASTH,w
//         movwf   FRAMEH
//         ; test format
//         btfss   LASTL,7
//         goto    TR24
//         ; low = 1bbb bbbb
//         rlf     FRAMEL
//         rrf     FRAMEH
//         rrf     FRAMEH
//         rrf     FRAMEL
//         goto    TR25
// TR24    ; low = 01bb bbbb
//         rlf     FRAMEL
//         rlf     FRAMEL
//         rrf     FRAMEH
//         rrf     FRAMEL
//         rrf     FRAMEH
//         rrf     FRAMEL
// TR25    movf    FRAMEH,w
//         andlw   7
//         movwf   FRAMEH

//         ; test for command frames
//         movf    FRAMEH,w
//         sublw   4
//         btfss   STATUS,Z   ; is it a CMD?
//         goto    TR30       ; no.
//         ; test for special command frames: 494-497 and 49c-49f
//         movf    FRAMEL,w
//         andlw   0xf4
//         sublw   0x94
//         btfss   STATUS,Z   ; is is a special command frame?
//         goto    TR26       ; no, 
//         ; handle the special frames
//         movf    FRAMEL,w
//         sublw   0x94   ; TDIS
//         btfss   STATUS,Z
//         goto    TR27S
//         bcf     ILFLG,TRENA
//         bcf     ILFLG,CTRL
//         bcf     ILFLG,SSRQ
//         goto    TR27X
// TR27S   movf	FRAMEL,w
//         sublw   0x95   ; COFI  - new 1.51
//         btfss   STATUS,Z
//         goto    TR27A
//         bsf     ILFLG,TRIDY
//         goto    TR27R
// TR27A   movf    FRAMEL,w
//         sublw   0x96   ; CON
//         btfss   STATUS,Z
//         goto    TR27B
//         bsf     ILFLG,TRENA
//         bsf     ILFLG,CTRL
//         goto    TR27X
// TR27B   movf    FRAMEL,w
//         sublw   0x97   ; COFF
//         btfss   STATUS,Z
//         goto    TR27C
//         bcf     ILFLG,TRIDY
// TR27R   bsf     ILFLG,TRENA
//         bcf     ILFLG,SSRQ
//         bcf     ILFLG,CTRL
//         goto    TR27X
// TR27C   movf    FRAMEL,w
//         sublw   0x9C   ; SSRQ
//         btfss   STATUS,Z
//         goto    TR27D
//         bsf     ILFLG,SSRQ
//         goto    TR27X
// TR27D   movf    FRAMEL,w
//         sublw   0x9D   ; CSRQ
//         btfss   STATUS,Z
//         goto    TR27X
//         bcf     ILFLG,SSRQ
// TR27X   ; send back ack to PC
//         movf    LASTL,w
//         call    sndserial
//         goto    TR40

// TR26    ; cmd:
//         btfsc   ILFLG,CTRL
//         goto    TR30
//         ; non-ctrl: send RFC
//         movlw   0x05
//         movwf   FRAMEH
//         clrf    FRAMEL
// TR30    
//         btfss   ILFLG,TRENA
//         goto    TR27X
//         goto    sendIL       ; send frame and returns to Main

// TR40    bsf     INTCON, GIE   ; enable intr
//         bsf     PORTA,LEDOFF   ; LED off
//         goto    Main        
       
// ; ----------------------------------------------------        
// ; send a frame to IL. Returns to Main as fast as possible
// sendIL
//         movf    CMCON,w  
//         bcf     PIR1, CMIF   ; clear comparator IT
// ; sndframe
// ; send a frame from FRAME
// MASKM   equ  1
// MASKP   equ  8
//         movf  FRAMEH,w
//         movwf FRREGH
//         movf  FRAMEL,w
//         movwf FRREGL

//         btfsc FRREGH,2
//         goto  sendf1

// ; send '0' sync bit
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         goto   sendf2

// sendf1
// ; send '1' sync bit
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
// sendf2  nop
//         movlw  0  ; 
//         movwf  PORTB
//         nop

//         movlw D'9'   ; loop for the next 9 bits
//         movwf CPTBIT
                      
// sendn       
//         rlf   FRREGL
//         rlf   FRREGH
//         btfsc FRREGH,2
//         goto  send1
//         nop
// ; send '0' bit
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         goto   send2

// send1
// ; send '1' bit
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         nop
// send2   nop
//         movlw  0  ; 
//         movwf  PORTB

//         decfsz CPTBIT
//         goto   sendn

// ; last bit, optimized for fix with the HP71B HPIL module rev 1A
        
//         nop
//         rlf   FRREGL
//         rlf   FRREGH
//         btfsc FRREGH,2
//         goto  send11
//         nop
        
// ; send '0' bit
//         movlw  MASKM  ; OUT-
//         movwf  PORTB
//         nop
//         movlw  MASKP  ; OUT+
//         goto   send21

// send11
// ; send '1' bit
//         movlw  MASKP  ; OUT+
//         movwf  PORTB
//         nop
//         nop
//         nop
//         movlw  MASKM  ; OUT-
// send21  movwf  PORTB
//         movlw  0  ; 
//         btfsc  PIR1, CMIF   ; check comparator IT
//         goto   startframe   ; start of frame received!
//         nop
//         movwf  PORTB
                
//         bsf    INTCON, GIE ; enable intr
//         bsf    PORTA,LEDOFF   ; LED off        
//         goto   Main

// startframe
//         movwf  PORTB
//         call   Intr1        ; immediately call intr (GIE set by RETFIE)
//         bsf    PORTA,LEDOFF   ; LED off        
//         bsf    ILFLG,FRAV   ; frame available
//         goto   Main
        

                

// ; ---------------------------------	
// ; send a byte to serial
// sndserial
//         btfss   PIR1,TXIF
//         goto    sndserial
//         movwf   TXREG
//         return
        

// ; ---------------------------------	

// decframe
// ; decode frame from samples in BUF (20-5f). Result in FRAME, set ILFLG,FRAV if frame found
// ; new version 1.5
//         movlw   BUF
//         movwf   FSR
//         movlw   40     ; 64 registers (128 samples)
//         movwf   CPTSAM
//         clrf    CPTBIT
//         clrf    DECFLG ; flags for decoding
//         bsf     DECFLG,fIDLE
// ; DECFLG flags: 
// fBIT    equ     0    ; fBIT must be bit 0
// fIDLE   equ     1    ; idle state
// fDET    equ     2    ; bit detected
// ; sample bit positions:
// fINp    equ     6
// fINm    equ     7
// fINp2   equ     2
// fINm2   equ     3

//         decf    FSR  ; 
// decfra1
//         incf    FSR  ; !

// ; find bits
// ; positive pulse? 
//         btfsc   INDF,fINp
//         goto    fndtr1
// ; negative pulse?
//         btfss   INDF,fINm
//         goto    fndtr2
// ; not in IDLE state ?        
//         btfss   DECFLG,fIDLE
//         goto    fndtr3

// fndtr0  ; same for the 2 other bits of current sample
//         btfsc   INDF,fINp2
//         goto    fndtr1b
//         btfss   INDF,fINm2
//         goto    fndtr2b
//         btfss   DECFLG,fIDLE
//         goto    fndtr3b
//         decfsz  CPTSAM
//         goto    decfra1
//         goto    fndtr6
        
// fndtr1  clrf    DECFLG         ; not idle state
// ;        bcf     DECFLG,fBIT    ; last positive pulse will mean '0' bit
//         goto    fndtr0

// fndtr2  clrf    DECFLG         ; not idle state
//         bsf     DECFLG,fBIT    ; last negative pulse will mean '1' bit
//         goto    fndtr0
        
// fndtr3                                
// ; check IDLE state after at least one pulse is found
// ; idle state found
//         btfss   DECFLG,fDET
//         goto    fndtr5
// fndtr4
// ; idle found 2 times: one bit found, store it
//         rrf     DECFLG,W
//         rlf     FRREGL
//         rlf     FRREGH
//         incf    CPTBIT
//         bsf     DECFLG,fIDLE
// fndtr5  bsf     DECFLG,fDET
//         goto    fndtr0

// fndtr1b clrf    DECFLG         ; not idle state
//         decfsz  CPTSAM
//         goto    decfra1
//         goto    fndtr6
// fndtr2b clrf    DECFLG         ; not idle state
//         bsf     DECFLG,fBIT 
//         decfsz  CPTSAM
//         goto    decfra1
//         goto    fndtr6
// fndtr3b btfss   DECFLG,fDET
//         goto    fndtr5b
// fndtr4b rrf     DECFLG,W
//         rlf     FRREGL
//         rlf     FRREGH
//         incf    CPTBIT
//         bsf     DECFLG,fIDLE
// fndtr5b bsf     DECFLG,fDET
//         decfsz  CPTSAM
//         goto    decfra1

// fndtr6
// ; end of samples. did we get exactly 11 bits?
//         movf    CPTBIT,W
//         sublw   D'11'
//         btfsc   STATUS,Z
//         goto    fndtr8              ; yes
//         btfsc   PORTB,FIX1A
//         goto    fndtr9
//         ; fix for HP71B HPIL module rev 1A
//         ; check if start of frame was detected during sending
//         btfss   ILFLG,FRAV
//         goto    fndtr9  ; no
// ; yes. 10 bits received?        
//         movf    CPTBIT,W
//         sublw   D'10'
//         btfss   STATUS,Z
//         goto    fndtr9              ; no
// ; check if last sent frame was a data        
//         btfsc   FRAMEH,2
//         goto    fndtr9
//         ; ok
//         bcf     FRREGH,2            ; set 1st bit to 0
//         ;
// fndtr8  bsf     ILFLG,FRAV          ; frame found
//         movf    FRREGH,w
//         andlw   7
//         movwf   FRAMEH
//         movf    FRREGL,w
//         movwf   FRAMEL
// fndtr9  return
       
 
// blinkLED
// ; make blink the LED
//         movlw   D'250'
//         call    LEDon
//         movlw   D'250'
//         call    Delay
        
//         movlw   D'250'
//         call    LEDon
//         movlw   D'250'
//         call    Delay

//         movlw   D'250'
//         call    LEDon

//         return   

// LEDon
//         movwf   CPTB
// LEDon1
//         clrwdt
//         bcf     PORTA,LEDOFF   ; LED on
//         call    Dly300us
//         bsf     PORTA,LEDOFF   ; LED off
//         call    Dly700us
//         decf    CPTB,F
//         btfss   STATUS,Z
//         goto    LEDon1
//         return



// ; ------------------------------------------------------------------------        
        
// ; delay of W ms
// Delay
//         movwf   CPTB
// d100ms  call    Dly1ms
//         clrwdt
//         decf    CPTB,F
//         btfss   STATUS,Z
//         goto    d100ms
//         return

// ; delay of 1 ms
// Dly1ms
//         call    Dly100us
//         call    Dly100us
//         call    Dly100us
// Dly700us      
//         call    Dly100us
//         call    Dly100us
//         call    Dly100us
//         call    Dly100us
// Dly300us      
//         call    Dly100us
//         call    Dly100us
//         call    Dly100us
//         return

// Dly100us                    ; delay 100us = 125 loops of 0.8us
//         movlw   D'131'      ; (from 131 to 256)
// d1ms    addlw   1           ; 1 cycle (0.2 us at 20MHz)
//         btfss   STATUS,Z    ; 1
//         goto    d1ms        ; 2
//         return


//         end


// */
