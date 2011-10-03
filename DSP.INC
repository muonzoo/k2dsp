{----------------------------------------------------------------------------}
{ DSP.INC                                                                    }
{ Standard definitions for ADSP-2181 memory and control registers            }
{ Author: Johan B. Forrer                                                    }
{ Date: 4/11/96                                                              }
{ Comments: Derived from examples that came with EZDSP package               }
{ modified April, 2003 for DSPX Module by L Johnson                          }
{----------------------------------------------------------------------------}

{Memory mapped control registers}
.const	IDMA_Control=		0x3fe0;
.const	BDMA_Int_Address=	0x3fe1;
.const	BDMA_Ext_Address=	0x3fe2;
.const	BDMA_Control=		0x3fe3;
.const	BDMA_Word_Count=	0x3fe4;
.const	PFLAG_Data=	  	0x3fe5;
.const	PFLAG_Control=		0x3fe6;
.const  SPORT1_Autobuf=         0x3fef;
.const  SPORT1_RFSDIV=          0x3ff0;
.const  SPORT1_SCLKDIV=         0x3ff1;
.const  SPORT1_Control_Reg=     0x3ff2;
.const  SPORT0_Autobuf=         0x3ff3;
.const  SPORT0_RFSDIV=          0x3ff4;
.const  SPORT0_SCLKDIV=         0x3ff5;
.const  SPORT0_Control_Reg=     0x3ff6;
.const  SPORT0_TX_Channels0=    0x3ff7;
.const  SPORT0_TX_Channels1=    0x3ff8;
.const  SPORT0_RX_Channels0=    0x3ff9;
.const  SPORT0_RX_Channels1=    0x3ffa;
.const  TSCALE=                 0x3ffb;
.const  TCOUNT=                 0x3ffc;
.const  TPERIOD=                0x3ffd;
.const  DM_Wait_Reg=            0x3ffe;
.const  System_Control_Reg=     0x3fff;

{-------------END DSP.INC----------------------------------------------------}

