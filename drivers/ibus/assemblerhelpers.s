        MODULE  asmhelpers

        PUBLIC  ibus_decode_proportinal_channels
        PUBLIC  ibus_calculate_checksum

        SECTION `.near_func.text`:CODE:NOROOT(0)

#include <vregs.inc>
        ; A is the rssi value
        ; X is indata pointer, Y is outdata pointer
        ; format of indata is [CH4 Hbits | CH3 Hbits | CH2 Hbits | CH1 Hbits][CH1 LB][CH2 LB][CH3 LB][CH4 LB]
        ; outformat is channel value + 1000 in little endian 
ibus_decode_proportinal_channels:
        PUSHW   X               ; save X before calculating rssi on in
        LDW     X,      #4
        MUL     X,      A
        ADDW    X,      #1000   
        SWAPW   X               ; convert to little endian
        LDW     (18,Y), X       ; save to channel list
        POPW    X
        LD      A,      (X)     ; save the high bits of every channel to virtual register 0
        LD      ?b0,    A
        MOV     ?b1,    #4      ; use virtual register 1 as downcounter for iterations
DECODE_LOOP:
        PUSHW   X               ; save X before overwriting it
        LDW     X,      (X)     ; load the lowbytes of the channel
        LD      A,      ?b0     ; extract the high bits of the channel
        PUSH    A               
        SRL     A               ; advance the high bits for next iteration
        SRL     A
        LD      ?b0,    A
        POP     A
        AND     A,      #3
        LD      XH,     A       ; overwrite garbage in channel high byte
        ADDW    X,      #1000   ; add offset of ibus channel value range 1000 - 2000
        SWAPW   X               ; convert to little endian
        LDW     (Y),    X       ; save to outbuffer
        POPW    X               ; restore X
        DEC     ?b1             ; check iteration value
        JREQ    DECODE_DONEZO     ; if donezo goto donezo
        INCW    Y               ; advance the outpointer for next iteration
        INCW    Y
        INCW    X               ; advance the inpointer for next iteration
        JP      DECODE_LOOP     
DECODE_DONEZO:
        RET                     ; DONEZO

     // calculate the checksum over all the variable data
     // uint16_t checksum = 0xFF9F;
     // for (uint8_t* pc = (uint8_t*)&buffer_; pc != (uint8_t*)&buffer_[10]; ++pc)
     // {
     //   checksum -= *pc;
     // }
     // swap_bytes(checksum);
     // X is address of buffer and return value for checksum
ibus_calculate_checksum:
        MOV     ?b0,    #$FC    ; initialize checksum with (0xFFFF - static values)
        LD      A,      #$1b
        MOV     ?b1,    #20     ; set up loop counter
CHECKSUM_LOOP:
        SUB     A,      (X)     ; calculaute checksum byte
        JRNC    CHECKSUM_NO_UV
        DEC     ?b0
CHECKSUM_NO_UV:
        DEC     ?b1
        JREQ    CHECKSUM_DONEZO
        INCW    X
        JP      CHECKSUM_LOOP
CHECKSUM_DONEZO:
        LD      XH,     A       ; save checksum in little endian to X
        LD      A,      ?b0
        LD      XL,     A
        RET
        
        END

