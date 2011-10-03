{----------------------------------------------------------------------------}
{ UART.INC                                                                   }
{ Variables for UART usage                                                   }
{ Author: Johan B. Forrer                                                    }
{ Date: 7/28/96                                                              }
{ Comments: Derived from EZ-KIT examples                                     }
{----------------------------------------------------------------------------}
.external   init_uart;          { UART initialize baudrate etc.              }
.external   out_char_ax1;       { UART output a character                    }
.external   get_char_ax1;       { UART wait & get input character            }
.external   get_char_ax1_to;    { UART wait & get input char. with timeout   }
.external   out_int_ar;         { UART wait & get input int with timeout     }
.external   get_int_ar_to;      { UART wait & get input int with timeout     }
.external   turn_rx_on;         { UART enable the rx section                 }
.external   turn_rx_off;        { UART disable the rx section                }
.external   process_a_bit;      { UART timer interrupt routine for RX and TX }
.external   led_cntr;           { UART led blinker counter at 3 * baud rate  }
.external   led_cntr_top;       { UART led blinker counter top               }
.external   flag_tx_ready;      { UART flag indicating UART is ready for new tx word }
.external   flag_rx_no_word;    { UART indicates a word is not in the user_rx_buffer }
.external   flag_rx_ready;      { UART flag indicating UART is doing new rx word     }
.external   char_ready;         { UART input character ready                 }
{----------------------------------------------------------------------------}

