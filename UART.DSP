.module/ram     UART;

{----------------------------------------------------------------------------}
{  Adapted from Analog Devices code examples by Johan Forrer,  July 1996.    }
{                                                                            }
{ Modified for KK7P DSPx Module by Lyle Johnson, KK7P                        }
{ January-April 2003                                                         }
{----------------------------------------------------------------------------
   ADSP-2101 Family Software UART                             UART.DSP
   --------------------------------------------------------------------
 
   Description:
    This uses FLAG_IN, FLAG_OUT and the TIMER of ADSP-218xN to interface to
    an asynchronous serial device such as the KDSP2.

        ADSP-218x  FLAG_OUT ----------> KDSP2 MCU UART RX

        ADSP-218x  FLAG_IN <----------  KDSP2 MCU UART TX

                       (ADSP-218x TIMER maintains baudrate)

    Parameters bits/word, baudrate, stopbits & parity are user-programmable.

    An RS-232 line driver chip can be used to electrically interface +3.3V
    logic levels to the RS-232 line voltage levels if required.

    The operation of the transmitter setup routine is completely
    independent on the the receiver setup routine operation.  Although
    both TX and RX use the same timer as a master clock source, the
    xmitted bits need not be in sync with the received bits. The default
    state of the reciever is OFF, so the "turn_rx_on" subroutine must be
    used to enable RX.

Calling Argument:
             for autobaud load the baud constant
        dm(baud_period)=(Proc_frequency/(3*Baudrate))-1

Useful Subroutines:
        init_uart       Must be called after system reset.
        get_char_ax1    Waits for RX input and returns with it in ax1.
        out_char_ax1    Waits for last TX output and transmits data from ax1.
        turn_rx_on      Must be called to enable the receipt of RX data.
        turn_rx_off     Can be used to ignore input RX data.

Useful Flag:
        DM(flag_rx_ready)   If this DM location is all ones it indicates that
                            the UART is ready to rx new word. If it is zero
                            then data is being received. Can be used for xon
                            xoff flow control.

Minor Modifications January-March 2003 by L Johnson for DSPx
-----------------------------------------------------------------------------}
 
.include <dsp.inc>;

.const CRYSTAL_FREQ_IN_kHZ=     40000;	        { for kk7p module }

.const  LEDRateGood=            14423;          { 14424 / 28800 ~= 1/2 S }
.const  LEDRateBad=             3606;           {  3606 / 28800 ~= 1/8 S }

{______The Constants below must be changed to modify uart parameters _______}

.const  tx_num_of_bits = 10;    { start bits + tx data bits + stop bits      }
.const  rx_num_of_bits = 8;     { rx data bits (start&stop bits not counted) }
.const  RX_BIT_ADD = 0x0100;    { = 1<<rx_num_of_bits                        }
.const  TX_BIT_ADD = 0xfe00;    { = 0xffff<<(tx data bits+1)                 }
.const  PERIOD = (CRYSTAL_FREQ_IN_kHZ * 2000 / (3 * 9600)) - 1; {  9600 bps  }

{----------------------------------------------------------------------------}
{                         UART entry points                                  }
{----------------------------------------------------------------------------}
.entry init_uart;         { UART initialize baudrate etc.                    }
.entry stop_uart;         { UART stop timer and mask interrupt.              }
.entry out_char_ax1;      { UART output a character                          }
.entry get_char_ax1;      { UART wait & get input character                  }
.entry get_char_ax1_to;   { UART wait & get input character with time out    }
.entry out_int_ar;        { UART wait & get input int with timeout           }
.entry get_int_ar_to;     { UART wait & get input int with timeout           }
.entry turn_rx_on;        { UART enable the rx section                       }
.entry turn_rx_off;       { UART disable the rx section                      }
.entry process_a_bit;     { UART timer interrupt routine for RX and TX       }

{----------------------------------------------------------------------------}
.global baud_period;      { fixed baud rate }
.global flag_rx_ready;
.global flag_tx_ready;
.global flag_rx_no_word;
.global char_ready;         { UART input character ready }
.global led_cntr;           { UART led blinker counter at 3 * baud rate }
.global led_cntr_top;       { UART led blinker counter top  }

{----------------------------------------------------------------------------}

.var/ram   flag_tx_ready;      { flag indicating UART is ready for new tx word }
.var/ram   flag_rx_ready;      { flag indicating UART is ready to rx new word}
.var/ram   flag_rx_stop_yet;   { flag tells that a rx stop bit is not pending}
.var/ram   flag_rx_no_word;    { indicates a word is not in the user_rx_buffer }
.var/ram   flag_rx_off;        { indicates a that the receiver is turned off }
.var/ram   timer_tx_ctr;       { divide by 3 ctr, timer is running @ 3x baudrate }
.var/ram   timer_rx_ctr;       { divide by 3 ctr, timer is running @ 3x baudrate }
.var/ram   user_tx_buffer;     { UART tx reg loaded by user before UART xmit }
.var/ram   user_rx_buffer;     { UART rx reg read by user after word is rcvd }
.var/ram   internal_tx_buffer; { formatted for serial word, adds start&stop bits }
                               { 'user_tx_buffer' is copied here before xmission }
.var/ram    internal_rx_buffer;
.var/ram    bits_left_in_tx;   { number of bits left in tx buffer (not yet clkd out) }
.var/ram    bits_left_in_rx;   { number of bits left to be rcvd (not yet clkd in) }
.var/ram    baud_period;       { loaded by autobaud routine                  }
.var/ram    char_ready;        { !0 when input character ready               }

.var/ram    led_cntr;          { led blinker counter at 3 * baud rate        }
.var/ram    led_cntr_top;      { led blinker counter top                     }

{-----------------------------------------------------------------------------
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : imask
 *  output : none
 *  destroy: ar, ax0, ay0
 *  keep   : none
 *  memory : none
 *
 *           dm (TSCALE) = 0
 *           dm (char_ready) = 0
 *           dm (selfTstRst)
 *           dm (led_cntr_top) =
 *           dm (led_cntr) =
 *           dm(flag_tx_ready) = 1
 *           dm(flag_rx_ready) = 1
 *           dm(flag_rx_stop_yet) = 1
 *           dm(flag_rx_no_word) = 1
 *           dm(flag_rx_off) = 1
 *
 *           flag_out
 *
 *  calls  : none
 *
 ----------------------------------------------------------------------------}
init_uart:
    ax0=0;
    dm(TSCALE)=ax0;             { decrement TCOUNT every instruction cycle   }
    dm(char_ready)=ax0;         { no input character ready                   }

    ax0 = 0;			{ dummy for selftest result                  }

    ar = LEDRateBad;            { led counter top: no error, 2 Hz.           }
    ay0 = LEDRateGood;
    none = pass ax0;
    if eq ar = pass ay0;        { led counter top: with error, 8 Hz.         }
    dm (led_cntr_top) = ar;     { set led counter top.                       }
    dm (led_cntr) = ar;         { clear LED blinker counter                  }

{----------------------------------------------------------------------------}
{ The uart may be set up to do an auto baud detect, in which case the bit    }
{ period must be saved in "baud_period"					     }
{----------------------------------------------------------------------------}
{   ax0=dm(baud_period);       }{ from autobaud or use constant: ax0=PERIOD; }
                                { ...and comment in the appropriate constant }
    ax0=PERIOD;                 { autobaud not used                          }
    dm(TCOUNT)=ax0;
    dm(TPERIOD)=ax0;            { interrupts generated at 3x baudrate        }

    ax0=1;
    dm(flag_tx_ready)=ax0;      { set the flags showing that UART is not busy}
    dm(flag_rx_ready)=ax0;
    dm(flag_rx_stop_yet)=ax0;
    dm(flag_rx_no_word)=ax0;
    dm(flag_rx_off)=ax0;        { rx section off                             }

    set flag_out;               { UART tx output is initialized to high      }

{ Note the System_control_Reg needs t be modified to use the UART !          } 
    ar = dm (System_Control_Reg);  { NEW !!!! }
    ay0 = b#1111101111111111;
    ar = ar and ay0;            { clear to enable FI, etc.                   }
    dm (System_Control_Reg) = ar;

{----------------------------------------------------------------------------}
{               Enable Timer Interrupt handling				     }
{----------------------------------------------------------------------------}
    ax0=imask;
    ay0=b#0000000001;
    ar=ax0 or ay0;
    imask=ar;                   { enable TIMER interrupt handling            }
    ena timer;                  { start timer now                            }

    rts;

{_________________________Stop UART    subroutine____________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : imask
 *  output : none
 *  destroy: ar, ay0
 *  keep   : none
 *  memory : flag_out
 *  calls  : none
 *
 ******************************************************************************}
stop_uart:
    dis timer;                  { start timer now }
    set flag_out;               { UART tx output is initialized to high }
    ifc = b#00000011111111;     { clear pending interrupt }
    nop;                        { wait for ifc latency }
    ar=imask;
    ay0=b#1111111110;
    ar=ar and ay0;
    imask=ar;                   { enable TIMER interrupt handling }
    rts;

{___________________Process_a_bit (TIMER interrupt routine)_________________

    This routine is the heart of the UART.  It is called every timer
    interrupt (i.e. 3x baudrate).  This routine will xmit one bit at a
    time by setting/clearing the FLAG_OUT pin of the ADSP-2101. This routine
    will then test if the uart is already receiving. If not it will test
    flagin (rx) for a start bit and place the uart in receive mode if true.
    If already in receive mode it will shift in one bit at a time by reading
    the FLAG_IN pin.  Since the internal timer is running at 3x baudrate,
    bits need only be transmitted/received once every 3 timer interrupts.
_____________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : none (second register bank)
 *  destroy: none
 *  keep   : none
 *  memory :
 *
 *           dm (usrRan)
 *           dm (led_cntr);
 *           dm (led_cntr_top)
 *
 *           dm(flag_tx_ready)
 *           dm(timer_tx_ctr)
 *           dm(internal_tx_buffer)
 *           dm(bits_left_in_tx)
 *           dm(flag_rx_off)
 *           dm(flag_rx_stop_yet)
 *           dm(timer_rx_ctr)
 *           dm(flag_rx_ready)
 *           dm(char_ready)
 *           dm(internal_rx_buffer)
 *           dm(flag_rx_no_word)
 *           dm(bits_left_in_rx)
 *
 *           flag_out
 *
 *  calls  : none
 *
 ******************************************************************************}
process_a_bit:

    ena sec_reg;                    { Switch to the background dreg set }

{-- LED blinker--}
    ar = dm (led_cntr);             { LED blinker counter }
    ax0 = dm (led_cntr_top);        { LED blinker counter top }
    ar = ar - 1;
{    if eq toggle fl1;     }          { toggle LED }
    if eq ar = pass ax0;            { reset counter }
    dm (led_cntr) = ar;             { save counter }
noled:

    ax0=dm(flag_tx_ready);     { if not in "transmit", go right to "receive" }
    none=pass ax0;
    if ne jump receiver;

{----------------------------------------------------------------------------}
{                        Transmitter Section                                 }
{----------------------------------------------------------------------------}
    ay0=dm(timer_tx_ctr);           { test timer ctr to see if a bit }
    ar=ay0-1;                       { is to be sent this time around }
    dm(timer_tx_ctr)=ar;            { if no bit is to be sent }
    if ne jump receiver;            { then decrement ctr and return }

    sr1=dm(internal_tx_buffer);     { shift out LSB of internal_tx_buffer }
    sr=lshift sr1 by -1 (hi);       { into SR1.  Test the sign of this bit }
    dm(internal_tx_buffer)=sr1;     { set or reset FLAG_OUT accordingly }
    ar=pass sr0;                    { this effectively clocks out the }
    if ge reset flag_out;           { word being xmitted one bit at a time }
    if lt set flag_out;             { LSB out first at FLAG_OUT. }

    ay0=3;                          { reset timer ctr to 3, i.e. next bit }
    dm(timer_tx_ctr)=ay0;           { will be sent after 3 timer interrupts }

    ay0=dm(bits_left_in_tx);        { number of bits left to be xmitted }
    ar=ay0-1;                       { is now decremented by one, }
    dm(bits_left_in_tx)=ar;         { indicating that one is now xmitted }
    if gt jump receiver;            { if no more bits left, then ready }

    ax0=1;                          { flag is set to true indicating }
    dm(flag_tx_ready)=ax0;          { a new word can now be xmitted }

{----------------------------------------------------------------------------}
{                         Receiver Section                                   }
{----------------------------------------------------------------------------}
receiver:
    ax0=dm(flag_rx_off);            { Test if receiver is turned on }
    none=pass ax0;
    if ne rti;

    ax0=dm(flag_rx_stop_yet);       { Test if finished with stop bit of }
    none=pass ax0;                  { last word or not. if finished then }
    if ne jump rx_test_busy;        { continue with check for receive. }

    ay0=dm(timer_rx_ctr);           { decrement timer ctr and test to see }
    ar=ay0-1;                       { if stop bit period has been reached }
    dm(timer_rx_ctr)=ar;            { if not return and wait }
    if ne rti;

    ax0=1;                          { if stop bit is reached then reset }
    dm(flag_rx_stop_yet)=ax0;       { to wait for next word }
    dm(flag_rx_ready)=ax0;
    dm (char_ready) = ax0;          { set to 1 indicates character available }

    ax0=dm(internal_rx_buffer);     { copy internal rx buffer }
    dm(user_rx_buffer)=ax0;         { to the user_rx_buffer }

    ax0=0;                          { indicated that a word is ready in }
    dm(flag_rx_no_word)=ax0;        { the user_rx_buffer }
    rti;

rx_test_busy:
    ax0=dm(flag_rx_ready);          { test rx flag, if rcvr is not busy }
    ar=pass ax0;                    { receiving bits then test for start.If it}
    if eq jump rx_busy;             { is busy, then clk in one bit at a time }

    if flag_in jump rx_exit;        { Test for start bit and return if none }

    ax0=0;
    dm(flag_rx_ready)=ax0;          { otherwise, indicate rcvr is now busy }
    dm(internal_rx_buffer)=ax0;     { clear out rcv register}

    ax0=4;                          { timer runs @ 3x baud rate, so rcvr }
    dm(timer_rx_ctr)=ax0;           { will only rcv on every 3rd interrupt }
                                    { initially this ctr is set to 4.  This }
                                    { will skip the start bit and will }
                                    { allow us to check FLAG_IN at the center }
                                    { of the received data bit }
    ax0=rx_num_of_bits;
    dm(bits_left_in_rx)=ax0;
rx_exit:
    rti;

rx_busy:
    ay0=dm(timer_rx_ctr);           { decrement timer ctr and test to see }
    ar=ay0-1;                       { if bit is to be rcvd this time around }
    dm(timer_rx_ctr)=ar;            { if not return, else receive a bit }
    if ne rti;

rcv:            { Shift in rx bit }
    ax0=3;                          { reset the timer ctr to 3 indicating }
    dm(timer_rx_ctr)=ax0;           { next bit is 3 timer interrupts later }

    ay0=RX_BIT_ADD;
    ar=dm(internal_rx_buffer);
    if not flag_in jump pad_zero;   { Test RX input bit and }
    ar=ar+ay0;                      { add in a 1 if hi }

pad_zero:
    sr=lshift ar by -1 (lo);        { Shift down to ready for next bit }
    dm(internal_rx_buffer)=sr0;

    ay0=dm(bits_left_in_rx);        { if there are more bits left to be rcvd }
    ar=ay0-1;                       { then keep UART in rcv mode }
    dm(bits_left_in_rx)=ar;         { and return }
    if gt rti;                      { if there are no more bits then.. }

                       { That was the last bit }
    ax0=3;                          { set timer to wait for middle of the }
    dm(timer_rx_ctr)=ax0;           { stop bit }
    ax0=0;                          { flag indicated that uart is waiting }
    dm(flag_rx_stop_yet)=ax0;       { for the stop bit to arrive }
    rti;

{___________________invoke_UART_transmit subroutine____________________

    This is the first step in the transmit process.  The user has now
    loaded 'user_tx_buffer' with the ascii code and has also invoked
    this routine.
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : none
 *  destroy: ar, ax0, sr
 *  keep   : none
 *  memory :
 *           dm(timer_tx_ctr)
 *           dm(bits_left_in_tx)
 *           dm(user_tx_buffer)
 *           dm(internal_tx_buffer)
 *           dm(flag_tx_ready)
 *  calls  : none
 *
__________________________________________________________________________}
invoke_UART_transmit:
    ax0=3;                          { initialize the timer decimator ctr }
    dm(timer_tx_ctr)=ax0;           { this divide by three ctr is needed }
                                    { since timer runs @ 3x baud rate }

    ax0=tx_num_of_bits;             { this constant is defined by the }
    dm(bits_left_in_tx)=ax0;        { user and represents total number of }
                                    { bits including stop and parity }
                                    { ctr is initialized here indicating }
                                    { none of the bits have been xmitted }
    sr1=0;
    sr0=TX_BIT_ADD;                 { upper bits are hi to end txmit with hi }
    ar=dm(user_tx_buffer);          { transmit register is copied into  }
    sr=sr or lshift ar by 1 (lo);   { the internal tx reg & left justified }
    dm(internal_tx_buffer)=sr0;     { before it gets xmitted }

    ax0=0;                          { indicate that the UART is busy }
    dm(flag_tx_ready)=ax0;
    rts;

{______________________get an input character______________________________
output:         ax1     (upper 8 bits are always cleared)
modifies:       ax0
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : ax1
 *  destroy: ax0
 *  keep   : none
 *  memory :
 *           dm(flag_rx_no_word)
 *           dm (char_ready)
 *           dm(user_rx_buffer)
 *           dm(flag_rx_no_word)
 *  calls  : none
 *
 ******************************************************************************}
get_char_ax1:
        ax0=dm(flag_rx_no_word);
        none=pass ax0;
        if ne jump get_char_ax1;       { if no rx word input, then wait }

get_char_ax1x:
 {       toggle fl1;            }        { toggle LED }
        dm (char_ready) = ax0;         { clear character ready flag }

        ax1=dm(user_rx_buffer);         { get received ascii character }
        ax0=1;
        dm(flag_rx_no_word)=ax0;        { word was read }
        rts;

{______________________get an input character with time out________________
output:         ax1     (all 16 bits set if timeout)
modifies:       ax0, af
time out and return 0xffff after 500 mS.
(15000 ticks / 3 == 5000 bps ~= 5000 / 9600  ~= .5 S)
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : ax1
 *  destroy: ax0
 *  keep   : none
 *  memory :
 *           dm(flag_rx_no_word)
 *           dm (char_ready)
 *           dm(user_rx_buffer)
 *           dm(flag_rx_no_word)
 *  calls  : none
 *
 ******************************************************************************}
get_char_ax1_to:
        ax1 = ar;                       { preserve ar }
        ar = 30000;                     { 3 ticks per bit; 9600 bps, time out }
                                        { at 1 sec ==> 9600 * 3 == 30000 }
get_char_ax1_to1:
        idle;                           { wait for timer interrupt }
        ar = ar - 1;
        if eq jump get_char_ax1_to2;    { time out now }

        ax0=dm(flag_rx_no_word);
        none=pass ax0;
        if ne jump get_char_ax1_to1;   { if no rx word input, then wait }
        ar = ax1;                       { restore ar }
        jump get_char_ax1x;             { read the character. }

get_char_ax1_to2:
        ar = ax1;                       { restore ar }
        ax1 = 0xffff;
        rts;

{______________________get a 16-bit integer with time out__________________
output:         ar
modifies:       ax0, ax1, si, sr0, sr1, ay0
astat:          ge if ok; lt if timed out
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : ar, (ax1)
 *  destroy: ay0, sr, si, (ax0)
 *  keep   : none
 *  memory :
 *          (
 *           dm(flag_rx_no_word)
 *           dm (char_ready)
 *           dm(user_rx_buffer)
 *           dm(flag_rx_no_word)
 *          )
 *  calls  : get_char_ax1_to
 *
 ******************************************************************************}
get_int_ar_to:
{   get hi 8 bits }
        call get_char_ax1_to;
        none = pass ax1;            { check time out }
        if lt rts;
        si = ax1;
        sr = lshift si by 8 (lo);   { shift to high byte }
        ay0 = sr0;

{   get lo 8 bits }
        call get_char_ax1_to;
        none = pass ax1;            { check time out }
        if lt rts;
        ar = ax1 or ay0;            { or in low byte }
        none = pass ax1;            { ax1 is >= 0; set ge for ok return code }
        rts;





{______________________output a character__________________________________
input:          ax1 (preserved)
modifies:       ax0, sr1, sr0, ar
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : ax1
 *  update : none
 *  output : none
 *  destroy: ax0, (ar, sr)
 *  keep   : none
 *  memory :
 *           dm(flag_tx_ready)
 *           dm(user_tx_buffer)
 *          (
 *           dm(timer_tx_ctr)
 *           dm(bits_left_in_tx)
 *           dm(user_tx_buffer)
 *           dm(internal_tx_buffer)
 *           dm(flag_tx_ready)
 *          )
 *  calls  : invoke_UART_transmit
 *
 ******************************************************************************}
out_char_ax1:
        ax0=dm(flag_tx_ready);
        none=pass ax0;
        if eq jump out_char_ax1;   { if tx word out still pending, then wait }
 {       toggle fl1;            }    { toggle LED }
        dm(user_tx_buffer)=ax1;
        call invoke_UART_transmit;      { send it out }
        rts;

{______________________output a 16-bit int_________________________________
input:          ar (preserved)
modifies:       ax0, ax1, sr1, sr0, ay0
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : ar
 *  update : none
 *  output : none
 *  destroy: sr, ax1, ay0, (ax0)
 *  keep   : none
 *  memory :
 *          (
 *           dm(flag_tx_ready)
 *           dm(user_tx_buffer)
 *          (
 *           dm(timer_tx_ctr)
 *           dm(bits_left_in_tx)
 *           dm(user_tx_buffer)
 *           dm(internal_tx_buffer)
 *           dm(flag_tx_ready)
 *          )
 *          )
 *
 *  calls  : out_char_ax1
 *
 ******************************************************************************}
out_int_ar:
        ay0 = ar;                  { save lo 8 bits }
        sr = lshift ar by -8 (lo); { shift hi 8 bits to lo 8 bits }
        ax1 = sr0;
        call out_char_ax1;         { send hi 8 bits }
        ax1 = ay0;
        call out_char_ax1;         { send lo 8 bits }
        rts;

{______________________enable the RX section________________________________
modifies:       ax0
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : none
 *  destroy: ax0 = 0
 *  keep   : none
 *  memory :
 *           dm(flag_rx_off)
 *  calls  : none
 *
 ******************************************************************************}
turn_rx_on:
        ax0=0;
        dm(flag_rx_off)=ax0;
        rts;

{______________________disable the RX section_______________________________
modifies:       ax0
___________________________________________________________________________}
{******************************************************************************
 *
 *
 *
 *  REGISTER USAGE SUMMARY:
 *
 *  input  : none
 *  update : none
 *  output : none
 *  destroy: ax0 = 1
 *  keep   : none
 *  memory :
 *           dm(flag_rx_off)
 *  calls  : none
 *
 ******************************************************************************}
turn_rx_off:
        ax0=1;
        dm(flag_rx_off)=ax0;
        rts;

{----------------------------------------------------------------------------}
.endmod;
