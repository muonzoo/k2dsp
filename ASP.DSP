.module/RAM/ABS=0       ASP;

{************************************************************************
 *
 * Audio Signal Processor
 *
 * Developed for use in the KDSP2 Project, adopted by Elecraft in 2003.
 *
 * NOTICE: Copyright (C) 2003 by Lyle V. Johnson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This module has its roots in the Audio Signal Processor by Johan
 * Forrer, KC7WW, Monroe OR, and borrow heavily from
 * Johan's ASP article which appeared in QEX for September, 1996.
 *
 * A great debt is also owed to the work of Bob Larkin in the DSP-10 project
 * from QST, September - November, 1999.
 *
 * The basic structure of Johan's code remains, and some initialization.
 * The CODEC roruitnes are replaced, BDMA filter updating is added, many
 * new commands are included.  Some sections are dropped entirely, and others
 * modified heavily.
 *
 * 17 July 2003 - version 3.18
 *     changed LMS_init to LMS_init_all to fix a bug in LMS beta/decay
 *     settings from KDSP2
 *
 ****************************************************************************
                             IMPORTANT!!!
  CODEC uses registers I5,L5,I6,L6, and M5 in Autobuffer mode. Leave intact!
-----------------------------------------------------------------------------}

.include    <dsp.inc>;

.GLOBAL rx_buf, tx_buf, sigout_L, sigout_R, SOFT_switch;
.EXTERNAL input_isr, input_init;
.EXTERNAL LMS_init, LMS, Denoise_init, LMS_init_all;
.EXTERNAL Denoise_switch, Autonotch_switch;
.EXTERNAL CW_switch, SSB_switch, DATA_switch;
.EXTERNAL beta; 		{ beta factor for denoising                  }
.EXTERNAL beta_aut;  		{ beta factor for autonotch                  }
.EXTERNAL decay;		{ decay factor for denoiser                  }
.EXTERNAL decay_aut;		{ decay factor for autonotcher               }
.EXTERNAL denoise_taps;		{ length of denoise filter                   }
.EXTERNAL denoise_tapsm1;	{ length of denoise filter-1                 }

.const FILTER_SIZE=128;	 { for CW/SSB/DATA filters and local dm declaration  }

.EXTERNAL denoiser_large_gain, denoiser_small_gain;	{ in LMS module      }
.EXTERNAL autonotch_large_gain, autonotch_small_gain;	{ in LMS module      }
.EXTERNAL cwfil_large_gain, cwfil_small_gain;		{ in ISR module      }
.EXTERNAL ssbfil_large_gain, ssbfil_small_gain;		{ in ISR module      }
.EXTERNAL datafil_large_gain, datafil_small_gain;	{ in ISR module      }

.EXTERNAL CW_coeffs;		{ location of  CW Filter		     }
.const bdma_cw_width_max=15;	{ initial settings }
.const bdma_cw_center_max=15;	

.EXTERNAL SSB_coeffs;		{ location of SSB Filter		     }
.const bdma_ssb_lower_max=11;	{ initial settings }
.const bdma_ssb_upper_max=15;	

.EXTERNAL DATA_coeffs;		{ location of Data Filter		     }
.const bdma_data_width_max=15;	{ initial settings }
.const bdma_data_center_max=15;	

.const firmware_version_hi=0x03;	{ XX part of xx.yy                   } 
.const firmware_version_lo=0x18;	{ YY part of xx.yy                   } 

.include <uart.inc>;

{----------------------------------------------------------------------------}
{                     Variables						     }
{----------------------------------------------------------------------------}

.var/dm/ram/circ	rx_buf[2];      /* L data + R data */
.var/dm/ram/circ        tx_buf[2];      /* R data + L data, note reversal */

.var/dm/ram/circ	temp_coeffs[FILTER_SIZE];   /* for BDMA load */


.VAR/DM/RAM  		sigout_L;
.VAR/DM/RAM  		sigout_R;
.var/dm/ram		filter_info, status_flag, ssb_fil, cw_fil, rtty_fil,
			aic_gain, standby_mode, SOFT_switch;

.var/dm/ram		bdma_cwfilter_width, bdma_cwfilter_center,
 			bdma_datafilter_width, bdma_datafilter_center,
			bdma_ssbfilter_low, bdma_ssbfilter_high,
			debug16, bdma_choice, cmd_flag,
			save_i0, save_l0, save_m0, save_i7, save_l7, save_m7,
			betaCmd, decayCmd, beta_autCmd, decay_autCmd;

.init tx_buf:   0x0000, 0x0000; 


{----------------------------------------------------------------------------}
{                     CODEC Initialization Constants			     }
{----------------------------------------------------------------------------}

{.const	C_LT_IN_VOL =	b#0000000100010111;}	{ Left Input Volume Control 0 dB }
{.const	C_LT_IN_VOL =	b#0000000100011001;}	{ Left Input Volume Control 3 dB }
.const	C_LT_IN_VOL =	b#0000000100010100;	{ Left Input Volume Control -4.5 dB 
			  |-----|||xx|---|
			  |     |||  |   +------ Input Gain LSB 11111=+12 dB, 1.5 dB steps
			  |     |||  +---------- Input Gain MSB 00000=-34.5 dB (0 dB shown)
			  |     ||+------------- Input Mute (1=mute)
			  |     |+-------------- LR Simultaneous Gain/Mute Update (1=enable)
			  +-----+--------------- Left Input Volume Control Register }

{.const	C_RT_IN_VOL =	b#0000001100010111;}	{ Right Input Volume Control 0 dB }
{.const	C_RT_IN_VOL =	b#0000001100011001;}	{ Right Input Volume Control 3 dB }
.const	C_RT_IN_VOL =	b#0000001100010100;	{ Right Input Volume Control -4.5 dB 
			  |-----|||xx|---|
			  |     |||  |   +------ Input Gain LSB 11111=+12 dB, 1.5 dB steps
			  |     |||  +---------- Input Gain MSB 00000=-34.5 dB (0 dB shown)
			  |     ||+------------- Input Mute (1=mute)
			  |     |+-------------- RL Simultaneous Gain/Mute Update (1=enable)
			  +-----+--------------- Right Input Volume Control Register }

{.const	C_LT_PHONE_VOL =b#0000010101111001;}	{ Left Headphone Volume Control 0 dB }
{.const	C_LT_PHONE_VOL =b#0000010101110110;}	{ Left Headphone Volume Control -3 dB }
.const	C_LT_PHONE_VOL =b#0000010101111111;	{ Left Headphone Volume Control 6 dB
			  |-----||||-----|
			  |     ||||     +------ Gain LSB 1111111=+6 dB, 1 dB steps
			  |     |||+------------ Gain MSB 0110000=-73 dB (mute) (0 dB shown)
			  |     ||+------------- Zero Cross Detect (1=mute)
			  |     |+-------------- LR Simultaneous Gain/Mute Update (1=enable)
			  +-----+--------------- Left Headphone Volume Control Register }

{.const	C_RT_PHONE_VOL =b#0000011101111001;}	{ Right Headphone Volume Control 0 dB }
{.const	C_RT_PHONE_VOL =b#0000011101110110;}	{ Right Headphone Volume Control -3 dB }
.const	C_RT_PHONE_VOL =b#0000011101111111;	{ Right Headphone Volume Control 6 dB
			  |-----||||-----|
			  |     ||||     +------ Gain LSB 1111111=+6 dB, 1 dB steps
			  |     |||+------------ Gain MSB 0110000=-73 dB (mute) (0 dB shown)
			  |     ||+------------- Zero Cross Detect (1=mute)
			  |     |+-------------- RL Simultaneous Gain/Mute Update (1=enable)
			  +-----+--------------- Right Headphone Volume Control Register }

.const	C_AUD_PATH =	b#0000100000010010;	{ Audio Path Control
			  |-----||||||||||
			  |     |||||||||+------ MIC Boost (0=0dB, 1=20 dB)
			  |     ||||||||+------- MIC Mute (1=muted) 
			  |     |||||||+-------- ADC Input Select (0=Line, 1=Mic)
			  |     ||||||+--------- Bypass (1=enabled)
			  |     |||||+---------- DAC Select (1=selected, 0= off)
			  |     ||||+----------- SideTone Enable (0 = disabled)
			  |     |||+------------ STA0 1= - 9 or -18 dB, 0= -6 or -12 dB
			  |     ||+------------- STA1 1= -12 or -18 dB, 0= -6 or -9 dB
			  |     |+-------------- STA2 1=   0 dB, 0= -6, -9, -12 or -18 dB
			  +-----+--------------- Audio Path Control Register }

.const	C_DIG_AUD_PATH =b#0000101000000000;	{ Digital Audio Path Control
			  |-----|xxxxx||||
			  |     |     |||+------ ADC High Pass Filter (1=enabled)
			  |     |     ||+------- DEEMP0 (1= 32 or 48 kHz)
			  |     |     |+-------- DEEMP1 (1= 44.1 or 48 kHz)
			  |     |     +--------- DAC Soft Mute (1=enabled)
			  +-----+--------------- Digital Audio Path Control Register }

{***}

.const	C_AUD_PATH_LOW =b#0000100000001010;	{ Audio Path Control
			  |-----||||||||||
			  |     |||||||||+------ MIC Boost (0=0dB, 1=20 dB)
			  |     ||||||||+------- MIC Mute (1=muted) 
			  |     |||||||+-------- ADC Input Select (0=Line, 1=Mic)
			  |     ||||||+--------- Bypass (1=enabled)
			  |     |||||+---------- DAC Select (1=selected, 0= off)
			  |     ||||+----------- SideTone Enable (0 = disabled)
			  |     |||+------------ STA0 1= - 9 or -18 dB, 0= -6 or -12 dB
			  |     ||+------------- STA1 1= -12 or -18 dB, 0= -6 or -9 dB
			  |     |+-------------- STA2 1=   0 dB, 0= -6, -9, -12 or -18 dB
			  +-----+--------------- Audio Path Control Register }


.const	C_POWER_LOW =	b#0000110001101110;	{ Power Down Control
			  |-----|x||||||||
			  |     | |||||||+------ Line Input   (1=OFF)
			  |     | ||||||+------- Mic Input    (1=OFF)
			  |     | |||||+-------- ADC          (1=OFF)
			  |     | ||||+--------- DAC          (1=OFF)
			  |     | |||+---------- Outputs      (1=OFF)
			  |     | ||+----------- Oscillator   (1=OFF)
			  |     | |+------------ Clock        (1=OFF)
			  |     | +------------- Device Power (1=OFF)
			  +-----+--------------- Power Down Control Register }

{***}

.const	C_POWER =	b#0000110000000010;	{ Power Down Control
			  |-----|x||||||||
			  |     | |||||||+------ Line Input   (1=OFF)
			  |     | ||||||+------- Mic Input    (1=OFF)
			  |     | |||||+-------- ADC          (1=OFF)
			  |     | ||||+--------- DAC          (1=OFF)
			  |     | |||+---------- Outputs      (1=OFF)
			  |     | ||+----------- Oscillator   (1=OFF)
			  |     | |+------------ Clock        (1=OFF)
			  |     | +------------- Device Power (1=OFF)
			  +-----+--------------- Power Down Control Register }

.const	C_DIG_FORMAT =	b#0000111001000001;	{ Digital Audio Interface Format
			  |-----|xx|||||||
			  |     |  ||||||+------ FOR0 Data Format
			  |     |  |||||+------- FOR1 (11 = DSP, 10= I2S, 01 MSB left aligned, 00 MSB right)
			  |     |  ||||+-------- IWL0 Input Word Length
			  |     |  |||+--------- IWL1 (11=32 bit, 10=24 bit, 01 = 20 bit, 00 = 16 bit)
			  |     |  ||+---------- DAC LR Phase (1= Right Channel LRCIN low, 0= high)
			  |     |  ||             in DSP Mode (1= MSB on 2nd Bclk / after LRCIN /, 0=1st Bclk / )
			  |     |  |+----------- DAC LR Swap (1=swap, 0=normal)
			  |     |  +------------ MS Mode (1=Master, 0=Slave)
			  +-----+--------------- Digital Audio Interface Format Register }

.const	C_SAMPLE_RATE =	b#0001000001011010;	{ Sample Rate Control, 16 kHz 18.432 crystal every third by 4 
			  |-----|x||||||||
			  |     | |||||||+------ Clock Mode (1=USB, 0=Normal)
			  |     | ||||||+------- Base Oversampling Rate (0=256, 1=384 in normal mode)
			  |     | |||||+-------- SR0 Sample Rate, see data sheet
			  |     | ||||+--------- SR1    shown is 16 kHz sample rate with
			  |     | |||+---------- SR2    18.432 MHz Xtal and 24 channels/frame
			  |     | ||+----------- SR3    for use with the adi dsp chip and cpld
			  |     | |+------------ CLKIN  (1=MCLK/2, 0=MCLK)
			  |     | +------------- CLKOUT (1=MCLK/2, 0=MCLK)
			  +-----+--------------- Sample Rate Control Register }

.const	C_DIG_ACT =	b#0001001000000001;	{ Digital Interface Activation
			  |-----|xxxxxxxx|
			  |     |        |+----- Activate Interface (1=active, 0=inactive)
			  +-----+--------------- Digital Interface Activation Register }

.const	C_RESET =	b#0001111000000000;	{ Reset
			  |-----||-------|
			  |     |+-------|+----- Must be all 0's for RESET
			  +-----+--------------- Reset Register }

.const  SPICLK=	0x0002;	{ PF data bits for SPI port to init CODEC }
.const  SPICS=	0x0004;
.const  SPIDAT=	0x0001;
.const	CODEC=	0x0000;	{ Dummy address, any will do in I/O space }

{----------------------------------------------------------------------------}
{            Interrupt vector table					     }
{----------------------------------------------------------------------------}
        jump start;	rti; rti; rti;		{00: reset }
        rti;		rti; rti; rti;		{04: IRQ2 }
        rti;		rti; rti; rti;		{08: IRQL1 }
        rti;		rti; rti; rti;		{0c: IRQL0 }
        rti;		rti; rti; rti;		{10: SPORT0 tx }
        jump input_isr; rti; rti; rti;		{14: SPORT0 rx }
			
        rti;		rti; rti; rti;		{18: IRQE }
{        jump bdma_irq; }
	rti;             rti; rti; rti;		{1c: BDMA }
        jump irq1isr;        rti; rti; rti;	{20: SPORT1 tx or IRQ1 }
        rti;                 rti; rti; rti;	{24: SPORT1 rx or IRQ0 }
        jump process_a_bit;  rti; rti; rti;	{28: timer }
        rti;                 rti; rti; rti;	{2c: power down }

{----------------------------------------------------------------------------}
{       ADSP 2181 intialization						     }
{----------------------------------------------------------------------------}
start:
        dis ints;
	set FL1;	{ turn on LED for diagnostics }


        ax0 = b#0000100000000000;	{  Shut down sport 0 }
	dm (System_Control_Reg) = ax0;

        i5 = ^rx_buf;
        l5 = %rx_buf;
        i6 = ^tx_buf;
        l6 = %tx_buf;
 
        m1 = 1;
        m5 = 1;

{***}

{ set various gains in LMS and ISR sections }

	ax1=0x21;			{ denoiser +6 dB }
	call figger_gain;
	dm(denoiser_large_gain)=ax0;
	dm(denoiser_small_gain)=ay0;

	ax1=0x10;			{ autonotch 0 dB gain }
	call figger_gain;
	dm(autonotch_large_gain)=ax0;
	dm(autonotch_small_gain)=ay0;

	ax1=0x10;			{ cw filter 0 dB gain }
	call figger_gain;
	dm(cwfil_large_gain)=ax0;
	dm(cwfil_small_gain)=ay0;

	ax1=0x10;			{ ssb filter 0 dB gain }
	call figger_gain;
	dm(ssbfil_large_gain)=ax0;
	dm(ssbfil_small_gain)=ay0;

	ax1=0x10;			{ data filter 0 dB gain }
	call figger_gain;
	dm(datafil_large_gain)=ax0;
	dm(datafil_small_gain)=ay0;	

{***}





{================== S E R I A L   P O R T   #0   S T U F F ==================}

        ax0 = b#0000110011010111;   dm (SPORT0_Autobuf) = ax0;
            {   |||!|-/!/|-/|/|+- receive autobuffering 0=off, 1=on
                |||!|  ! |  | +-- transmit autobuffering 0=off, 1=on
                |||!|  ! |  +---- | receive m?
                |||!|  ! |        | m5
                |||!|  ! +------- ! receive i?
                |||!|  !          ! i0
                |||!|  !          !
                |||!|  +========= | transmit m?
                |||!|             | m6
                |||!+------------ ! transmit i?
                |||!              ! i1
                |||!              !
                |||+============= | BIASRND MAC biased rounding control bit
                ||+-------------- 0
                |+--------------- | CLKODIS CLKOUT disable control bit
                +---------------- 0
            }

        ax0 = 0;    dm (SPORT0_RFSDIV) = ax0;              {   RFSDIV = SCLK Hz/RFS Hz - 1 }
        ax0 = 0;    dm (SPORT0_SCLKDIV) = ax0;		   {   SCLK = CLKOUT / (2  (SCLKDIV + 1) }

        ax0 = b#1000000000001111;   dm (SPORT0_Control_Reg) = ax0;
            {   multichannel
                ||+--/|!||+/+---/ | number of bit per word - 1
                |||   |!|||       | = 15
                |||   |!|||       |
                |||   |!|||       |
                |||   |!||+====== ! 0=right just, 0-fill; 1=right just, signed
                |||   |!||        ! 2=compand u-law; 3=compand A-law
                |||   |!|+------- receive framing logic 0=pos, 1=neg
                |||   |!+-------- transmit data valid logic 0=pos, 1=neg
                |||   |+========= RFS 0=ext, 1=int
                |||   +---------- multichannel length 0=24, 1=32 words
                ||+-------------- | frame sync to occur this number of clock
                ||                | cycle before first bit
                ||                |
                ||                |
                |+--------------- ISCLK 0=ext, 1=int
                +---------------- multichannel 0=disable, 1=enable
            }
            {   non-multichannel
                |||!|||!|||!+---/ | number of bit per word - 1
                |||!|||!|||!      | = 15
                |||!|||!|||!      |
                |||!|||!|||!      |
                |||!|||!|||+===== ! 0=right just, 0-fill; 1=right just, signed
                |||!|||!||+------ ! 2=compand u-law; 3=compand A-law
                |||!|||!|+------- receive framing logic 0=pos, 1=neg
                |||!|||!+-------- transmit framing logic 0=pos, 1=neg
                |||!|||+========= RFS 0=ext, 1=int
                |||!||+---------- TFS 0=ext, 1=int
                |||!|+----------- TFS width 0=FS before data, 1=FS in sync
                |||!+------------ TFS 0=no, 1=required
                |||+============= RFS width 0=FS before data, 1=FS in sync
                ||+-------------- RFS 0=no, 1=required
                |+--------------- ISCLK 0=ext, 1=int
                +---------------- multichannel 0=disable, 1=enable
            }


        ax0 = b#1001001001001001;   dm (SPORT0_TX_Channels0) = ax0; { every third 32/16 ok }
            {   ^15          00^   transmit word enables: channel # == bit # }
        ax0 = b#0000000000100100;   dm (SPORT0_TX_Channels1) = ax0; { every third 32/16 ok }
            {   ^31          16^   transmit word enables: channel # == bit # }
        ax0 = b#1001001001001001;   dm (SPORT0_RX_Channels0) = ax0; { every third 32/16 ok }
            {   ^15          00^   receive word enables: channel # == bit # }
        ax0 = b#0000000000100100;   dm (SPORT0_RX_Channels1) = ax0; { every third 32/16 ok }
            {   ^31          16^   receive word enables: channel # == bit # }


{============== S Y S T E M   A N D   M E M O R Y   S T U F F ==============}
        ax0 = b#0000111111111111;   dm (DM_Wait_Reg) = ax0;
            {   |+-/+-/+-/+-/+-/- ! IOWAIT0
                ||  |  !  |       !
                ||  |  !  |       !
                ||  |  !  +------ | IOWAIT1
                ||  |  !          |
                ||  |  !          |
                ||  |  +--------- ! IOWAIT2
                ||  |             !
                ||  |             !
                ||  +------------ | IOWAIT3
                ||                |
                ||                |
                |+=============== ! DWAIT
                |                 !
                |                 !
                +---------------- 0
            }


        ax0 = b#1101000000000000;   dm(PFLAG_Data) = ax0;
            {   |+-/||||+------/- ! PFTYPE
                ||  |||||         !
                ||  ||||+---------!
                ||  |||+--------- | CMSEL PM
                ||  ||+---------- | CMSEL DM
                ||  |+----------- ! CMSEL BM
                ||  +------------ | CMSEL IOM
                |+=============== ! BMWAIT
                |                 ! 0xD=162.5 nSec
                |                 !
                +---------------- 0
            }


        ax0 = b#0001000000000000;   dm (System_Control_Reg) = ax0;
            {   +-/!||+-----/+-/- | program memory wait states
                |  !|||           | 0
                |  !|||           |
                |  !||+---------- 0
                |  !||            0
                |  !||            0
                |  !||            0
                |  !||            0
                |  !||            0
                |  !||            0
                |  !|+----------- SPORT1 1=serial port, 0=FI, FO, IRQ0, IRQ1,..
                |  !+------------ SPORT1 1=enabled, 0=disabled
                |  +============= SPORT0 1=enabled, 0=disabled
                +---------------- 0
                                  0
                                  0
            }


	ifc = b#00000011111111;         { clear pending interrupt }
	icntl = b#00010;	
	    {     ||||+- | IRQ0: 0=level, 1=edge
		  |||+-- | IRQ1: 0=level, 1=edge
		  ||+--- | IRQ2: 0=level, 1=edge
		  |+---- 0
		  |----- | IRQ nesting: 0=disabled, 1=enabled
	    }

        mstat = b#1000000;
            {     ||||||+- | Data register bank select
                  |||||+-- | FFT bit reverse mode (DAG1)
                  ||||+--- | ALU overflow latch mode, 1=sticky
                  |||+---- | AR saturation mode, 1=saturate, 0=wrap
                  ||+----- | MAC result, 0=fractional, 1=integer
                  |+------ | timer enable
                  +------- | GO MODE
            }

{----------------------------------------------------------------------------}
{                  TLV320AIC23 Codec intialization			     }
{----------------------------------------------------------------------------}

 	ax0=C_RESET;		call CODEC_CMD;	{ Reset }
	ax0=C_LT_IN_VOL;	call CODEC_CMD;	{ Left Input Volume Control }
	ax0=C_RT_IN_VOL;	call CODEC_CMD;	{ Right Input Volume Control }
	ax0=C_LT_PHONE_VOL;	
	dm(aic_gain)=ax0;	call CODEC_CMD;	{ Left Headphone Volume Control }
	ax0=C_RT_PHONE_VOL;	call CODEC_CMD;	{ Right Headphone Volume Control }
	ax0=C_AUD_PATH;		call CODEC_CMD;	{ Audio Path Control }
	ax0=C_DIG_AUD_PATH;	call CODEC_CMD;	{ Digital Audio Path Control }
	ax0=C_POWER;		call CODEC_CMD;	{ Power Down Control }
	ax0=C_DIG_FORMAT;	call CODEC_CMD;	{ Digital Audio Interface Format }
	ax0=C_SAMPLE_RATE;	call CODEC_CMD;	{ Sample Rate Control }
	ax0=C_DIG_ACT;		call CODEC_CMD;	{ Digital Interface Activation }

	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

 
        ifc = b#00000011111111;		{ clear any pending interrupt }
        nop;				{ allow ifc command to take effect }

        imask = b#0000100100;		{ enable rx0 interrupt }
            {     |||||||||+ | timer
                  ||||||||+- | SPORT1 rec or IRQ0
                  |||||||+-- | SPORT1 trx or IRQ1
                  ||||||+--- | BDMA
                  |||||+---- | IRQE
                  ||||+----- | SPORT0 rec
                  |||+------ | SPORT0 trx
                  ||+------- | IRQL0
                  |+-------- | IRQL1
                  +--------- | IRQ2
            }

{----------------------------------------------------------------------------}
{                  User program initialization                               }
{----------------------------------------------------------------------------}
	call input_init;		{ pointers and initial values }
{*** v 3.18 change }
	call LMS_init_all;		{ turns off notch, denoise, taps=91 }
{*** end of v 3.18 change }
        call init_uart;			{ intialize uart routine }
        call turn_rx_on;		{ turn on uart }

	ax0=0x08; dm(bdma_cwfilter_center)=ax0;	{ set BDMA cwfilter 800 Hz ctr }
	ax0=0x09; dm(bdma_cwfilter_width)=ax0;	{ set BDMA cwfilter 500 Hz wid }
	ax0=0x0B; dm(bdma_datafilter_center)=ax0;	{ set BDMA datafilter 2100 Hz ctr }
	ax0=0x02; dm(bdma_datafilter_width)=ax0;	{ set BDMA datafilter 200 Hz wid }
	ax0=0x0A; dm(bdma_ssbfilter_high)=ax0;	{ set BDMA ssbfilter 2500 Hz hi }
	ax0=0x03; dm(bdma_ssbfilter_low)=ax0;	{ set BDMA ssbfilter 200 Hz low }

	ax0=0x89; dm(cw_fil)=ax0;		{ show current cw filter setting }
	ax0=0xA3; dm(ssb_fil)=ax0;		{ show current ssb filter setting }
	ax0=0xB2; dm(rtty_fil)=ax0;		{ show current rtty filter setting }

	ax0=0x30; dm(status_flag)=ax0;		{ taps=91, no auto, no denoise, no mode }
	ax0=0xff; dm(filter_info)=ax0;		{ no mode set }
	ax0=0x00; dm(cmd_flag)=ax0;		{ no command "escape" received }
	dm(standby_mode)=ax0;			{ we are not in standby mode }		

{----------------------------------------------------------------------------}
{***}	reset FL0; {diagnostic }
        ena ints;

{----------------------------------------------------------------------------}
{  Idle loop. Nothing really happens here. Processing happens in CODEC ISR   }
{----------------------------------------------------------------------------}
again:  
	idle(128);		{ slow down clock to conserve power }

        call get_char_ax1; 	{ get command character }

	ax0=dm(cmd_flag);	{ two byte cmd second byte? }
	ar=PASS ax0;		
	if NE jump cmd_2;	{ yes, do second byte command }

{ it is a 1-byte command, or the first byte of a two-byte command }

{----------------------------------------------------------------------------}
{  Command                  Action			                     }
{----------------------------------------------------------------------------}
{     d         Denoiser OFF                                                 }
{     D         Denoiser ON                                                  }
{     n         autoNotch OFF                                                }
{     N         autoNotch ON                                                 }
{     b         Enable CW SOFT filter                    also bdma target    }
{     c         Enable CW filter                         also bdma target    }
{     s         Enable SSB filter   			 also bdma target    }
{     r         Enable RTTY filter                       also bdma target    }
{     x         Reset: denoiser, autonotch, CW                               }
{     t         DeNoise taps=31                                              }
{     u         DeNoise taps=41                                              }
{     v         DeNoise taps=51                                              }
{     w         DeNoise taps=61                                              }
{     0         CW Mode 800 Hz Ctr 500 Hz wide                               }
{     1         CW Mode 800 Hz Ctr 300 Hz wide                               }
{     2         CW Mode 800 Hz Ctr 100 Hz wide                               }
{     A         SSB Mode 200 Hz Low 2500 Hz High                             }
{     B         SSB Mode 200 Hz Low 2500 Hz High, autoNotch ON               }
{     C         SSB Mode 200 Hz Low 2500 Hz High, autoNotch and Denoiser ON  }
{     P         RTTY Mode 2100 Hz Ctr 200 Hz wide                            }
{     Q         RTTY Mode 1600 Hz Ctr 300 Hz wide                            }
{     R         RTTY Mode 10000 Hz Ctr 100 Hz wide                           }
{     >         BDMA CW Load higher center freq                              }
{     <		BDMA CW Load lower center freq                               }
{     .         BDMA CW Load wider filter                                    }
{     ,         BDMA CW Load narrower filter                                 }
{     :         BDMA DATA Load higher center freq                            }
{     "		BDMA DATA Load lower center freq                             }
{     '         BDMA DATA Load wider filter                                  }
{     ;         BDMA DATA Load narrower filter                               }
{    ^[         BDMA SSB Load higher upper freq                              }
{    ^]		BDMA SSB Load lower upper freq                               }
{     [         BDMA SSB Load lower lower freq                               }
{     ]         BDMA SSB Load higher lower freq                              }
{     (         BDMA Soft Load higher center freq                            }
{     )		BDMA Soft Load lower center freq                             }
{     =         BDMA Soft Load wider filter                                  }
{     _         BDMA Soft Load narrower filter                               }
{     ?         query current status                                         }
{     V         query DSP firmware revision                                  }
{     S         query denoiser beta and decay                                }
{     T         query autonotch beta and decay                               }
{     +         Increase AIC Gain                                            }
{     -         Decrease AIC Gain                                            }
{									     }
{     Z		place unit in low-power bypass mode			     }
{     z		place unit in normal operating mode			     }
{									     }
{----------------------------------------------------------------------------}
{  Command      This is the action for the first byte of a two byte command  }
{----------------------------------------------------------------------------}
{   0x80-BF    Directly set status (mode, denoiser, autonotch, taps          }
{              Second byte will set mode-indicated filter                    }
{   0xC1       Set DeNoiser Beta - second byte will be argument              }
{   0xC2       Set DeNoiser Decay - second byte will be argument             }
{   0xC4       Set AutoNotch Beta - second byte will be argument             }
{   0xC8       Set AutoNotch Decay - second byte will be argument            }
{   0xD0       Set DeNoiser Gain - second byte will be argument              }
{   0xD1       Set AutoNotch Gain - second byte will be argument             }
{   0xD2       Set CW Filter Gain - second byte will be argument             }
{   0xD3       Set SSB Filter Gain - second byte will be argument            }
{   0xD4       Set Data Filter Gain - second byte will be argument           }
{   0xD5       Set CODEC Gain - second byte will be argument                 }
{									     }
{----------------------------------------------------------------------------}

{ need to check the specific cases before the general case of MSB being set  }

	ay1=0xC1;			{ denoiser beta }
        none = ax1 - ay1;
        if eq jump prep_denoiser_beta;

	ay1=0xC2;			{ denoiser decay }
        none = ax1 - ay1;
        if eq jump prep_denoiser_decay;

	ay1=0xC4;			{ autonotch beta }
        none = ax1 - ay1;
        if eq jump prep_notch_beta;

	ay1=0xC8;			{ autonotch decay }
        none = ax1 - ay1;
        if eq jump prep_notch_decay;

	ay1=0xD0;			{ denoiser gain }
        none = ax1 - ay1;
        if eq jump prep_denoiser_gain;

	ay1=0xD1;			{ autonotch gain }
        none = ax1 - ay1;
        if eq jump prep_autonotch_gain;

	ay1=0xD2;			{ cw filter gain }
        none = ax1 - ay1;
        if eq jump prep_cw_filter_gain;

	ay1=0xD3;			{ ssb filter gain }
        none = ax1 - ay1;
        if eq jump prep_ssb_filter_gain;

	ay1=0xD4;			{ data filter gain }
        none = ax1 - ay1;
        if eq jump prep_data_filter_gain;

	ay1=0xD5;			{ codec gain }
        none = ax1 - ay1;
        if eq jump prep_direct_codec_gain;

{ now check the general case of MSB set = direct setting of status & filter  }

	ay1=0x80;			{ Direct Status Set }
        none = ax1 AND ay1;
        if ne jump set_status_direct;

{ none of the above, so it is a single byte command }

        ay1 = 0x64;  			{ d }
        none = ax1 - ay1;
        if eq jump denoiser_off;

        ay1 = 0x44;  			{ D }
        none = ax1 - ay1;
        if eq jump denoiser_on;

        ay1 = 0x6E;  			{ n }
        none = ax1 - ay1;
        if eq jump notch_off;
	
        ay1 = 0x4E;  			{ N }
        none = ax1 - ay1;
        if eq jump notch_on;
	
        ay1 = 0x62;  			{ b }
        none = ax1 - ay1;
        if eq jump do_SOFT_fil;

        ay1 = 0x63;  			{ c }
        none = ax1 - ay1;
        if eq jump do_CW_fil;

        ay1 = 0x3F;  			{ ? }
        none = ax1 - ay1;
        if eq jump status_query;

        ay1 = 0x56;  			{ V }
        none = ax1 - ay1;
        if eq jump version_query;

        ay1 = 0x73;  			{ s }
        none = ax1 - ay1;
        if eq jump do_SSB_fil;

        ay1 = 0x72;  			{ r }
        none = ax1 - ay1;
        if eq jump do_RTTY_fil;

        ay1 = 0x78;  			{ x }
        none = ax1 - ay1;
        if eq jump resets;

        ay1 = 0x58;  			{ X }
        none = ax1 - ay1;
        if eq jump resets;

        ay1 = 0x31;  			{ 1     CW 800@500 }
        none = ax1 - ay1;
        if eq jump do_CW_wide;

        ay1 = 0x32;  			{ 2     CW 800@300 }
        none = ax1 - ay1;
        if eq jump do_CW_med;

        ay1 = 0x33;  			{ 3     CW 800@100 }
        none = ax1 - ay1;
        if eq jump do_CW_nar;

        ay1 = 0x41;  			{ A     SSB }
        none = ax1 - ay1;
        if eq jump do_SSB_only;

        ay1 = 0x42;  			{ B     SSB+NOTCH }
        none = ax1 - ay1;
        if eq jump do_SSB_notch;

        ay1 = 0x43;  			{ C     SSB+NOTCH+DENOISE }
        none = ax1 - ay1;
        if eq jump do_SSB_notch_denoise;

        ay1 = 0x51;  			{ Q     RTTY 2100@200 }
        none = ax1 - ay1;
        if eq jump do_RTTY_high;

        ay1 = 0x52;  			{ R     RTTY 1700@300 }
        none = ax1 - ay1;
        if eq jump do_RTTY_med;

        ay1 = 0x53;  			{ S     RTTY 1000@100 }
        none = ax1 - ay1;
        if eq jump do_RTTY_low;

        ay1 = 0x74;  			{ t     DeNoise Shorter }
        none = ax1 - ay1;
        if eq jump DeNoise_taps31;

        ay1 = 0x75;  			{ u     DeNoise Normal? }
        none = ax1 - ay1;
        if eq jump DeNoise_taps51;

        ay1 = 0x76;  			{ v     DeNNoise Longer }
        none = ax1 - ay1;
        if eq jump DeNoise_taps71;

        ay1 = 0x77;  			{ w     DeNoise Longest }
        none = ax1 - ay1;
        if eq jump DeNoise_taps91;

        ay1 = 0x28;  			{ (     BDMA Soft Higher Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_soft_higher_freq;

        ay1 = 0x29;  			{ )     BDMA Soft Lower Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_soft_lower_freq;

        ay1 = 0x3d;  			{ =     BDMA Soft Wider Filter }
        none = ax1 - ay1;
        if eq jump BDMA_soft_wider;

        ay1 = 0x5f;  			{ _     BDMA Soft Narrower Filter }
        none = ax1 - ay1;
        if eq jump BDMA_soft_narrower;

        ay1 = 0x3e;  			{ >     BDMA CW Higher Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_cw_higher_freq;

        ay1 = 0x3c;  			{ <     BDMA CW Lower Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_cw_lower_freq;

        ay1 = 0x2e;  			{ .     BDMA CW Wider Filter }
        none = ax1 - ay1;
        if eq jump BDMA_cw_wider;

        ay1 = 0x2c;  			{ ,     BDMA CW Narrower Filter }
        none = ax1 - ay1;
        if eq jump BDMA_cw_narrower;

        ay1 = 0x3a;  			{ :     BDMA DATA Higher Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_data_higher_freq;

        ay1 = 0x22;  			{ "     BDMA DATA Lower Filter Freq }
        none = ax1 - ay1;
        if eq jump BDMA_data_lower_freq;

        ay1 = 0x3b;  			{ ;     BDMA DATA Wider Filter }
        none = ax1 - ay1;
        if eq jump BDMA_data_wider;

        ay1 = 0x27;  			{ '     BDMA DATA Narrower Filter }
        none = ax1 - ay1;
        if eq jump BDMA_data_narrower;

        ay1 = 0x7b;  			{ ^[    BDMA SSB Higher Upper Freq }
        none = ax1 - ay1;
        if eq jump BDMA_ssb_higher_freq;

        ay1 = 0x7d;  			{ ^]    BDMA SSB Lower Upper Freq }
        none = ax1 - ay1;
        if eq jump BDMA_ssb_lower_freq;

        ay1 = 0x5b;  			{ [     BDMA SSB Lower Lower Freq }
        none = ax1 - ay1;
        if eq jump BDMA_ssb_wider;

        ay1 = 0x5d;  			{ ]     BDMA SSB Higher Lower  Freq }
        none = ax1 - ay1;
        if eq jump BDMA_ssb_narrower;

        ay1 = 0x54;  			{ S     Query DeNoiser Beta and Decay }
        none = ax1 - ay1;
        if eq jump query_denoiser;

        ay1 = 0x55;  			{ T     Query AutoNotch Beta and Decay }
        none = ax1 - ay1;
        if eq jump query_autonotch;

        ay1 = 0x5a;  			{ Z     Place unit in low-power standby mode }
        none = ax1 - ay1;
        if eq jump set_low_power;

        ay1 = 0x7a;  			{ z     Place unit in normal operating mode }
        none = ax1 - ay1;
        if eq jump set_normal_power;

        ay1 = 0x2b;  			{ +     Increase aic gain }
        none = ax1 - ay1;
        if eq jump aic_gain_increase;

        ay1 = 0x2d;  			{ -     Decrease aic gain }
        none = ax1 - ay1;
        if eq jump aic_gain_decrease;


command_err:
	set FL1;			{ LED on }
	ax1=0xFF;			{  send 0xffff }
        call out_char_ax1; 
	ax1=0xFF;			
        call out_char_ax1; 
	jump again;

command_ok:
	reset FL1;			{ LED off }
	ax1=dm(status_flag);		{ output status flag to controller }
        call out_char_ax1; 
	ax1=dm(filter_info);		{ output filter parameters }
	call out_char_ax1;
	jump again;



{----------------------------------------------------------------------------}
{                     aic audio gain control	  			     }
{----------------------------------------------------------------------------}

aic_gain_decrease:
	ax0=dm(aic_gain);	{ get present value }
	ay0=0x7F;		{ mask off gain bits }
	ar=ax0 and ay0;
	if EQ jump command_ok;		{ don't wrap around! }

	ay0=1;			{ decrease gain }
	ar=ar - ay0;
	ay0=0x0500;		{ address within AIC for gain control }
	ar=ar + ay0;
	dm(aic_gain)=ar;
	ax0=ar;		call CODEC_CMD; {update gain }
	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

	jump command_ok;

aic_gain_increase:

	ax0=dm(aic_gain);	{ get present value }
	ay0=0x7F;		{ mask off gain bits }
	ar=ax0 and ay0;
	ar=ar-ay0;		{ is gain already at maximum ?}
	if EQ jump command_ok;		{ don't wrap around! }

	ay0=1;
	ar=ax0 + ay0;		{ increase gain }

{	ay0=0x0500;	}	{ address within AIC for gain control }
{	ar=ar + ay0;	}
	dm(aic_gain)=ar;
	ax0=ar;		call CODEC_CMD; {update gain }
	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

	jump command_ok;


{----------------------------------------------------------------------------}
{                     power control commands	  			     }
{----------------------------------------------------------------------------}

set_low_power:

	ax0=imask;
	ay0=0xFFDF;
	ar=ax0 and ay0;
	imask=ar;

	ax0=C_AUD_PATH_LOW;	call CODEC_CMD;	{ Audio Path Control }
	ax0=C_POWER_LOW;	call CODEC_CMD;	{ Power Down Control }

	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

        ifc = b#00000000100000;     { clear any pending rx0 interrupt }

	ax0=imask;
	ay0=0x0020;			{ enable rx0 interrupt }
	ar=ax0 or ay0;
	imask=ar;

	dis timer;			{ wait for an input char to restart}
	ax0=0xffff;			{ set standby mode flag }
	dm(standby_mode)=ax0;
	idle(128);			{ wait for uart receive char }
    set FL0; {diagnostic }
{	jump again;}
	idle(128);

set_normal_power:

	ax0=imask;
	ay0=0xFFDF;
	ar=ax0 and ay0;
	imask=ar;

 	ax0=C_RESET;		call CODEC_CMD;	{ Reset }
	ax0=C_POWER;		call CODEC_CMD;	{ Power Down Control }
	ax0=C_LT_IN_VOL;	call CODEC_CMD;	{ Left Input Volume Control }
	ax0=C_RT_IN_VOL;	call CODEC_CMD;	{ Right Input Volume Control }
	ax0=dm(aic_gain);	{C_LT_PHONE_VOL;}	
{	dm(aic_gain)=ax0;}	call CODEC_CMD;	{ Left Headphone Volume Control }
	ax0=C_RT_PHONE_VOL;	call CODEC_CMD;	{ Right Headphone Volume Control }
	ax0=C_AUD_PATH;		call CODEC_CMD;	{ Audio Path Control }
	ax0=C_DIG_AUD_PATH;	call CODEC_CMD;	{ Digital Audio Path Control }
	ax0=C_POWER;		call CODEC_CMD;	{ Power Down Control }
	ax0=C_DIG_FORMAT;	call CODEC_CMD;	{ Digital Audio Interface Format }
	ax0=C_SAMPLE_RATE;	call CODEC_CMD;	{ Sample Rate Control }
	ax0=C_DIG_ACT;		call CODEC_CMD;	{ Digital Interface Activation }
	ax0=C_AUD_PATH;		call CODEC_CMD;	{ Audio Path Control }

	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

 
        ifc = b#00000000100000;     { clear any pending rx0 interrupt }

	ax0=0;				{ clear standby mode flag }
	dm(standby_mode)=ax0;

	ax0=imask;
	ay0=0x20;			{ enable rx0 interrupt }
	ar=ax0 or ay0;
	imask=ar;

	jump command_ok;

{----------------------------------------------------------------------------}
{                     1st of 2 byte control commands  			     }
{----------------------------------------------------------------------------}
set_status_direct:
	ay1=0x83; 		{ mask mode bits, keep msb set }
	ar=ax1 AND ay1;
	dm(cmd_flag)=ar;	{ now set for second byte processing }

{ mode }
        none = ar - ay1;
        if eq jump set_rtty_mode;
	ay1=0x82;
        none = ar - ay1;
        if eq jump set_ssb_mode;
	ay1=0x81;
	none = ar - ay1;
	if eq jump set_cw_mode;

set_soft_mode:
	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar;
	ar=1; dm(CW_switch)=ar; dm(SOFT_switch)=ar;
	jump check_denoise_length;

set_cw_mode:
	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(CW_switch)=ar;
	jump check_denoise_length;

set_ssb_mode:
	ar=0; dm(CW_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(SSB_switch)=ar;
	jump check_denoise_length;

set_rtty_mode:
	ar=0; dm(CW_switch)=ar; dm(SSB_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(DATA_switch)=ar;

{ if denoiser length doesn't change then skip LMS init }
{ must do this before checking bits as init resets notch and denoise }

check_denoise_length:
	ay1=0x30;		{ mask denoiser length }
	ar=ax1 AND ay1;		{ get new value }
	ax0=dm(status_flag);	{ get current status value }
	af=ax0 AND ay1;		{ mask denoiser length }
	none=ar-af;		{ are they the same ?}
	if EQ jump check_denoise_state;	{ jump if yes }

{ change denoiser length - not the same }
	none=PASS ar;		{ see if 31}
	if EQ jump change_to_31;

	none=ar-ay1;		{ see if 91 }
	if EQ jump change_to_91;

	ay1=0x10;
	none=ar-ay1;		{ see if 71 }
	if EQ jump change_to_71;

change_to_51:
	call Denoise_init;
	ar=51;
	dm(denoise_taps)=ar;
	ar=50;
	dm(denoise_tapsm1)=ar;
	jump check_denoise_state;

change_to_31:
	call Denoise_init;
	ar=31;
	dm(denoise_taps)=ar;
	ar=30;
	dm(denoise_tapsm1)=ar;
	jump check_denoise_state;

change_to_71:
	call Denoise_init;
	ar=71;
	dm(denoise_taps)=ar;
	ar=70;
	dm(denoise_tapsm1)=ar;
	jump check_denoise_state;

change_to_91:
	call Denoise_init;	{ 91 taps is the default }

check_denoise_state:
	ay1=0x08;		{ mask denoiser bit }
	none=ax1 AND ay1;
	if EQ jump turn_denoiser_off;
	ar=0xFFFF;		{ turn it on }
	jump set_denoiser_switch;

turn_denoiser_off:
	ar=0x00;		{ turn it off }

set_denoiser_switch:
	dm(Denoise_switch)=ar;

check_autonotch_state:
	ay1=0x04;		{ mask autonotch bit }
	none= ax1 AND ay1;
	if EQ jump turn_autonotch_off;
	ar=0xFFFF;		{ turn it on }
	jump set_autonotch_switch;

turn_autonotch_off:
	ar=0x00;		{ turn it off }

set_autonotch_switch:
	dm(Autonotch_switch)=ar;
	ay1=0x3F;		{ drop upper two bits for status update }
	ar=ax1 AND ay1;
	dm(status_flag)=ar;	{ update old status value to new status value }

	jump again;		{ don't echo any response yet }

prep_denoiser_gain:		{ the prep is the same for all of these }
prep_autonotch_gain:
prep_cw_filter_gain:
prep_ssb_filter_gain:
prep_data_filter_gain:
prep_denoiser_beta:
prep_denoiser_decay:
prep_notch_beta:
prep_notch_decay:
prep_direct_codec_gain:
	dm(cmd_flag)=ax1;	{ now set for second byte processing }
	jump again;		{ don't echo any response yet }

{----------------------------------------------------------------------------}
{                     2nd of 2 byte control commands  			     }
{                                                                            }
{ ax0 has command flag which indicates what to do with argument              }
{     0x80 = reserved, mode = 0                                              }
{     0x81 = cw filter value                                                 }
{     0x82 = ssb filter value                                                }
{     0x83 = rtty filter value                                               }
{     0xC1 = denoiser beta value                                             }
{     0xC2 = denoiser decay value                                            }
{     0xC4 = autonotch beta value                                            }
{     0xC8 = autonotch decay value                                           }
{     0xD0 = denoiser gain value                                             }
{     0xD1 = autonotch gain value                                            }
{     0xD2 = cw filter gain value                                            }
{     0xD3 = ssb filter gain value                                           }
{     0xD4 = rtty filter gain value                                          }
{     0xD5 = codec gain value                                                }
{                                                                            }
{ ax1 has command argument                                                   }
{                                                                            }
{----------------------------------------------------------------------------}
cmd_2:
	ay1=0x00; dm(cmd_flag)=ay1;	{ clear flag for next time }

	ay1=0x80;			{ set soft cw filter }
        none = ax0 - ay1;
        if eq jump set_soft_filter;

	ay1=0x81;			{ set steep cw filter }
        none = ax0 - ay1;
        if eq jump set_cw_filter;

	ay1=0x82;			{ set ssb filter }
        none = ax0 - ay1;
        if eq jump set_ssb_filter;

	ay1=0x83;			{ set rtty filter }
        none = ax0 - ay1;
        if eq jump set_rtty_filter;

	ay1=0xC1;			{ denoiser beta }
        none = ax0 - ay1;
        if eq jump set_denoiser_beta;

	ay1=0xC2;			{ denoiser decay }
        none = ax0 - ay1;
        if eq jump set_denoiser_decay;

	ay1=0xC4;			{ autonotch beta }
        none = ax0 - ay1;
        if eq jump set_notch_beta;

	ay1=0xC8;			{ autonotch decay }
        none = ax0 - ay1;
        if eq jump set_notch_decay;

	ay1=0xD0;			{ denoiser gain }
        none = ax0 - ay1;
        if eq jump set_denoiser_gain;

	ay1=0xD1;			{ autonotch gain }
        none = ax0 - ay1;
        if eq jump set_autonotch_gain;

	ay1=0xD2;			{ cw filter gain }
        none = ax0 - ay1;
        if eq jump set_cwfilter_gain;

	ay1=0xD3;			{ ssb filter gain }
        none = ax0 - ay1;
        if eq jump set_ssbfilter_gain;

	ay1=0xD4;			{ data filter gain }
        none = ax0 - ay1;
        if eq jump set_datafilter_gain;

	ay1=0xD5;			{ codec gain }
        none = ax0 - ay1;
        if eq jump set_codec_gain;

	jump command_err;		{ no match, why are we here? }

{----------------------------------------------------------------------------}
{ !!! need to add range checking !!!                                         }
{----------------------------------------------------------------------------}
set_soft_filter:
	ay0=0xF0;			{ mask off upper 4 bits }
	ar=ax1 AND ay0;
	sr=LSHIFT ar by -4(LO);		{ move to lower 4 bits }
	dm(bdma_cwfilter_center)=sr0;
	ay0=0x0F;			{ mask off lower 4 bits }
	ar=ax1 AND ay0;
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_soft_setup;		{ and load new coeffs }

set_cw_filter:
	ay0=0xF0;			{ mask off upper 4 bits }
	ar=ax1 AND ay0;
	sr=LSHIFT ar by -4(LO);		{ move to lower 4 bits }
	dm(bdma_cwfilter_center)=sr0;
	ay0=0x0F;			{ mask off lower 4 bits }
	ar=ax1 AND ay0;
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_cw_setup;		{ and load new coeffs }

set_ssb_filter:
	ay0=0xF0;			{ mask off upper 4 bits }
	ar=ax1 AND ay0;
	sr=LSHIFT ar by -4(LO);		{ move to lower 4 bits }
	dm(bdma_ssbfilter_high)=sr0;
	ay0=0x0F;			{ mask off lower 4 bits }
	ar=ax1 AND ay0;
	dm(bdma_ssbfilter_low)=ar;
	jump BDMA_ssb_setup;		{ load new coeffs }

set_rtty_filter:
	ay0=0xF0;			{ mask off upper 4 bits }
	ar=ax1 AND ay0;
	sr=LSHIFT ar by -4(LO);		{ move to lower 4 bits }
	dm(bdma_datafilter_center)=sr0;
	ay0=0x0F;			{ mask off lower 4 bits }
	ar=ax1 AND ay0;
	dm(bdma_datafilter_width)=ar;
	jump BDMA_data_setup;		{ and load new coeffs }

set_notch_beta:
	dm(beta_autCmd)=ax1;

	ENA M_MODE;			{ set MAC to integer mode }
	mx0=ax1;			{ beta_autCmd=sqrt(LMS notch beta) }
	my0=mx0;
	mr=mx0*my0 (UU);		{ decompress data }
	DIS M_MODE;			{ reset MAC to fractional mode }
	sr=lshift mr0 by -2 (lo);	{ Divide by 4 }
	dm(beta_aut)=sr0;
	ay0=0xff;			{ mask lower byte }
	ar=sr0 AND ay0;
	dm(filter_info)=ar;
	sr=LSHIFT sr0 by -8(HI);	{ now fetch upper byte }
	dm(status_flag)=sr0;
	jump command_ok;		{ output updated final parameter }

set_denoiser_beta:
	dm(betaCmd)=ax1;
	ENA M_MODE;			{ set MAC to integer mode }
	mx0=ax1;			{ betaCmd=sqrt(LMS denoiser beta) }
	my0=mx0;
	mr=mx0*my0 (UU);		{ decompress data }
	DIS M_MODE;			{ reset MAC to fractional mode }
	sr=LSHIFT mr0 by -2 (LO);	{ reduce to desired range }
	dm(beta)=sr0;		
	ay0=0xff;			{ mask lower byte }
	ar=sr0 AND ay0;
	dm(filter_info)=ar;
	sr=LSHIFT sr0 by -8(LO);	{ now fetch upper byte }
	dm(status_flag)=sr0;

	jump command_ok;		{ output updated final parameter }

set_notch_decay:
	dm(decay_autCmd)=ax1;		{ save commanded value }
	sr0=ax1;
	sr=LSHIFT sr0 by 4(LO);		{ result = value * 16 }
	ay0=32735;			{ constant }
	ar=ay0-sr0;			{ decay = constant - result }
	dm(decay_aut)=ar;		{ update notch decay in LMS }

	dm(filter_info)=ar;		{ send lower byte }
	sr=LSHIFT ar by -8(LO);		{ now fetch upper byte }
	dm(status_flag)=sr0;

	jump command_ok;		{ output updated final parameter }

set_denoiser_decay:
	dm(decayCmd)=ax1;		{ save commanded value }
	sr0=ax1;
	sr=LSHIFT sr0 by 4(LO);		{ result = value * 16 }
	ay0=32735;			{ constant }
	ar=ay0-sr0;			{ decay = constant - result }
	dm(decay)=ar;			{ update denoiser decay in LMS }

	dm(filter_info)=ar;		{ send lower byte }
	sr=LSHIFT ar by -8(LO);		{ now fetch upper byte }
	dm(status_flag)=sr0;

	jump command_ok;		{ output updated final parameter }


set_denoiser_gain:
	call figger_gain;
	dm(denoiser_large_gain)=ax0;
	dm(denoiser_small_gain)=ay0;

	jump command_ok;

set_autonotch_gain:
	call figger_gain;
	dm(autonotch_large_gain)=ax0;
	dm(autonotch_small_gain)=ay0;

	jump command_ok;

set_cwfilter_gain:
	call figger_gain;
	dm(cwfil_large_gain)=ax0;
	dm(cwfil_small_gain)=ay0;

	jump command_ok;

set_ssbfilter_gain:
	call figger_gain;
	dm(ssbfil_large_gain)=ax0;
	dm(ssbfil_small_gain)=ay0;

	jump command_ok;

set_datafilter_gain:
	call figger_gain;
	dm(datafil_large_gain)=ax0;
	dm(datafil_small_gain)=ay0;

	jump command_ok;

set_codec_gain:			{ need to add range checking! }
	ay0=0x7F;		{ mask off gain bits }
	ar=ax1 and ay0;
	ay0=0x0580;		{ codec internal address for gain }
	ar=ar or ay0;		{ now we have the complete value }
	dm(aic_gain)=ar;	{ update memorized value for bypass mode }

	ax0=ar;		call CODEC_CMD; {update gain }
	ax0=SPICS       +SPICLK;	io(CODEC)=ax0;	{ set CPLD for 32/16 kHz sampling ok }

	jump command_ok;

{----------------------------------------------------------------------------}
{                            Query commands  				     }
{----------------------------------------------------------------------------}
status_query:				{ returns current status and filter }
	ar=dm(SSB_switch);
	none=pass ar;
	if EQ jump checkDataMode;
	ax0=0x02;			{ mode is SSB }
	ar=dm(ssb_fil);			{ update filter for current SSB }
	dm(filter_info)=ar;
	jump setQueryMode;

checkDataMode:
	ar=dm(DATA_switch);
	none=pass ar;
	if EQ jump checkSOFTMode;
	ax0=0x03;			{ mode is Data }
	ar=dm(rtty_fil);		{ update filter for current Data }
	dm(filter_info)=ar;
	jump setQueryMode;

checkSOFTMode:
	ar=dm(SOFT_switch);
	none=pass ar;
	if EQ jump checkCWMode;
	ax0=0x00;
	ar=dm(cw_fil);			{ update filter for current CW }
	dm(filter_info)=ar;
	jump setQueryMode;

checkCWMode:
	ax0=0x01;
	ar=dm(cw_fil);			{ update filter for current CW }
	dm(filter_info)=ar;

setQueryMode:
	ar=dm(Autonotch_switch);	{ check if autonotch is on }
	none=pass ar;
	if EQ jump checkQueryDenoise;
	ay0=0x04;			{ set autonotch bit }
	ar=ax0 + ay0;
	ax0=ar;

checkQueryDenoise:
	ar=dm(Denoise_switch);
	none=pass ar;
	if EQ jump checkQueryDenoiseTaps;
	ay0=0x08;
	ar=ax0 + ay0;			{ set denoise bit }
	ax0=ar;

checkQueryDenoiseTaps:
	ar=dm(denoise_taps);		{ get current taps setting }
	ay0=31;				{ check if 31 }
	af=ar-ay0;
	if NE jump checkQueryTaps51;	
	jump checkQueryFilter;		{ bits=0 for taps=31, already done }

checkQueryTaps51:
	ay0=51;				{ check if 51 }
	af=ar-ay0;
	if NE jump checkQueryTaps71;
	ay0=0x10;			{ 0x10 means taps=51 }
	ar=ax0 + ay0;
	ax0=ar;
	jump checkQueryFilter;		{ complete status flag and check filters }
	
checkQueryTaps71:
	ay0=71;				{ check if 71 }
	af=ar-ay0;
	if NE jump checkQueryTaps91;
	ay0=0x10;			{ 0x20 means taps=71 }
	ar=ax0 + ay0;
	ax0=ar;
	jump checkQueryFilter;		{ complete status flag and check filters }

checkQueryTaps91:			{ it must be 91 }
	ay0=0x30;			{ 0x30 means taps=91 }
	ar=ax0 + ay0;
	ax0=ar;

checkQueryFilter:
	dm(status_flag)=ax0;		{ status flag is complete }
	jump command_ok;

version_query:
	reset FL1;			{ LED off }
	ax1=firmware_version_hi;	{ output integer part xx }
        call out_char_ax1; 
	ax1=firmware_version_lo;	{ output fractional part .yy }
	call out_char_ax1;
	jump again;

query_denoiser:
	ax0=dm(betaCmd); dm(status_flag)=ax0;	{ beta factor for denoiser   }
	ax0=dm(decayCmd); dm(filter_info)=ax0;	{ decay factor for denoiser  }
	jump command_ok;

query_autonotch:
	ax0=dm(beta_autCmd); dm(status_flag)=ax0;	{ beta factor for autonotch  }
	ax0=dm(decay_autCmd); dm(filter_info)=ax0;	{ decay factor for autonotch }
	jump command_ok;

{----------------------------------------------------------------------------}
{                         LMS control commands  			     }
{----------------------------------------------------------------------------}
denoiser_off:
	ar=0x0000;
	dm(Denoise_switch)=ar;
	ax0=0xF7;			{ clear denoise bit }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	dm(status_flag)=ar;
	jump command_ok;

denoiser_on:
	ar=0xFFFF;
	dm(Denoise_switch)=ar;
	ax0=0x08;			{ set denoise bit }
	ay0=dm(status_flag);
	ar=ax0 OR ay0;
	dm(status_flag)=ar;
	jump command_ok;

notch_off:
	ar=0x0000;
	dm(Autonotch_switch)=ar;
	ax0=0xFB;			{ clear autonotch bit }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	dm(status_flag)=ar;
	jump command_ok;

notch_on:
	ar=0xFFFF;
	dm(Autonotch_switch)=ar;
	ax0=0x04;			{ set autonotch bit }
	ay0=dm(status_flag);
	ar=ax0 OR ay0;
	dm(status_flag)=ar;
	jump command_ok;


DeNoise_taps31:				{ !resets autonotch! }
	call Denoise_init;
	ar=31;
	dm(denoise_taps)=ar;
	ar=30;
	dm(denoise_tapsm1)=ar;
	ax0=0xCF;			{ set taps=31 }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	dm(status_flag)=ar;
	jump denoiser_on;

DeNoise_taps51:				{ !resets autonotch! }
	call Denoise_init;
	ar=51;
	dm(denoise_taps)=ar;
	ar=50;
	dm(denoise_tapsm1)=ar;
	ax0=0xCF;			{ set taps=51 }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	ay0=0x10;
	ar=ar OR ay0;
	dm(status_flag)=ar;
	jump denoiser_on;

DeNoise_taps71:				{ !resets autonotch! }
	call Denoise_init;
	ar=71;
	dm(denoise_taps)=ar;
	ar=70;
	dm(denoise_tapsm1)=ar;
	ax0=0xCF;			{ set taps=71 }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	ay0=0x20;
	ar=ar OR ay0;
	dm(status_flag)=ar;
	jump denoiser_on;

DeNoise_taps91:				{ !resets autonotch! }
	call Denoise_init;
	ar=91;
	dm(denoise_taps)=ar;
	ar=90;
	dm(denoise_tapsm1)=ar;
	ax0=0xCF;			{ set taps=91 }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	ay0=0x30;
	ar=ar OR ay0;
	dm(status_flag)=ar;
	jump denoiser_on;

{----------------------------------------------------------------------------}
{                         CW filter control commands  			     }
{----------------------------------------------------------------------------}
do_CW_fil:
	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(CW_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x01;			{ mode = cw }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;
	ar=dm(cw_fil);
	dm(filter_info)=ar;
	jump command_ok;

do_SOFT_fil:
	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar;
	ar=1; dm(CW_switch)=ar; dm(SOFT_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; {ay1=0x00;}			{ mode = cw SOFT}
	ar=ar AND ay0; {ar=ar OR ay1;}
	dm(status_flag)=ar;
	ar=dm(cw_fil);
	dm(filter_info)=ar;
	jump command_ok;

do_SSB_fil:
	ar=0; dm(CW_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(SSB_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x02;			{ mode = ssb }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;
	ar=dm(ssb_fil);
	dm(filter_info)=ar;
	jump command_ok;

do_RTTY_fil:
	ar=0; dm(CW_switch)=ar; dm(SSB_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(DATA_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x03;			{ mode = rtty }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;
	ar=dm(rtty_fil);
	dm(filter_info)=ar;
	jump command_ok;

resets:
	ar=0;
	dm(CW_switch)=ar;
	dm(SSB_switch)=ar;
	dm(Denoise_switch)=ar;
	dm(Autonotch_switch)=ar;
	call LMS_init;
	ax0=0x30; dm(status_flag)=ax0;		{ taps=91, no auto, no denoise, no mode }
	ax0=0xff; dm(filter_info)=ax0;		{ no mode set }

	jump command_ok;

{----------------------------------------------------------------------------}
{             	BDMA Soft CW filter control commands  			     }
{----------------------------------------------------------------------------}


BDMA_soft_higher_freq:
	ax0=dm(bdma_cwfilter_center);	{ get current center freq }
	ay0=bdma_cw_center_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_cwfilter_center)=ar;
	jump BDMA_soft_setup;		{ and load new coeffs }

BDMA_soft_lower_freq:
	ax0=dm(bdma_cwfilter_center);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_cwfilter_center)=ar;
	jump BDMA_soft_setup;		{ and load new coeffs }

BDMA_soft_wider:
	ax0=dm(bdma_cwfilter_width);	{ get current center freq }
	ay0=bdma_cw_width_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_soft_setup;		{ and load new coeffs }

BDMA_soft_narrower:
	ax0=dm(bdma_cwfilter_width);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_soft_setup;		{ and load new coeffs }

BDMA_soft_setup:
	mx0=dm(bdma_cwfilter_center);
	my0=bdma_cw_width_max+1;	{ steps between centers }
	mr=mx0*my0(UU);
	ay0=dm(bdma_cwfilter_width);	{ add width }
	ar=mr0+ay0; ar=ar+ay0;
	sr=LSHIFT ar by 7(LO);		{ 0x100 bytes per filter step}

	ay0=0x3fff;			{ mask low order 14 bits }
	ar=sr0 AND ay0;			{ this is bdma address within page }
	dm(BDMA_Ext_Address)=ar;
	sr=LSHIFT sr0 by -6 (LO);	{ drop low order 14 bits }
{***}	ay0=0x1400;			{ base page >> 2 }
	ar=sr0+ay0;			{ this is bdma page }
	ay0=0xff00;			{ mask off leftover address bits }
	ar=ar AND ay0;
	ay0=1;				{ MCR=0, BDIR=read, BTYPE=16 bit DM }
	ar=ar OR ay0;
	dm(BDMA_Control)=ar;

	ax0=^temp_coeffs;		{ point to destination in dm }
	dm(BDMA_Int_Address)=ax0;

	ax0=FILTER_SIZE;		{ set up transfer length and start transfer }
	dm(BDMA_Word_Count)=ax0;

	ar=dm(bdma_cwfilter_center);	{ update cw_fil info while transfer taking place }
	sr=LSHIFT ar by 4(LO);		{ shift to upper nibble of lower byte }
	ay0=dm(bdma_cwfilter_width);
	ax0=0x0F;
	af=ax0 AND ay0;
	ar=sr0 OR af;
	ay0=0xFF;
	ar=ar AND ay0;
	dm(cw_fil)=ar;
	dm(filter_info)=ar;

	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar;
	ar=1; dm(CW_switch)=ar; dm(SOFT_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; {ay1=0x00;}		{ mode = soft cw }
	ar=ar AND ay0; {ar=ar OR ay1;}
	dm(status_flag)=ar;

check_soft_bdma:			{ wait for transfer to complete }
	ax0=dm(BDMA_Word_Count);
	ar=PASS ax0;
	if NE jump check_soft_bdma;

	dis ints;

{ save index registers and point to coefficients }
	ax0=i7; dm(save_i7)=ax0; i7=^CW_coeffs;

	jump bdma_coeff_update;

{----------------------------------------------------------------------------}
{             	BDMA CW filter control commands  			     }
{----------------------------------------------------------------------------}


do_CW_wide:
	ar=0x09;			{ width = 500 Hz }
	dm(bdma_cwfilter_width)=ar;
	jump do_CW_1;

do_CW_med:
	ar=0x05;			{ width = 300 Hz }
	dm(bdma_cwfilter_width)=ar;
	jump do_CW_1;

do_CW_nar:
	ar=0x01;			{ width = 100 Hz }
	dm(bdma_cwfilter_width)=ar;
do_CW_1:
	ar=0;
	dm(SSB_switch)=ar;
	dm(DATA_switch)=ar;
	dm(SOFT_switch)=ar;
	ar=1;
	dm(CW_switch)=ar;
	ar=0x08;			{ center = 800 Hz }
	dm(bdma_cwfilter_center)=ar;
	jump BDMA_cw_setup;		{ load new coeffs }

BDMA_cw_higher_freq:
	ax0=dm(bdma_cwfilter_center);	{ get current center freq }
	ay0=bdma_cw_center_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_cwfilter_center)=ar;
	jump BDMA_cw_setup;		{ and load new coeffs }

BDMA_cw_lower_freq:
	ax0=dm(bdma_cwfilter_center);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_cwfilter_center)=ar;
	jump BDMA_cw_setup;		{ and load new coeffs }

BDMA_cw_wider:
	ax0=dm(bdma_cwfilter_width);	{ get current center freq }
	ay0=bdma_cw_width_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_cw_setup;		{ and load new coeffs }

BDMA_cw_narrower:
	ax0=dm(bdma_cwfilter_width);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_cwfilter_width)=ar;
	jump BDMA_cw_setup;		{ and load new coeffs }

BDMA_cw_setup:
	mx0=dm(bdma_cwfilter_center);
	my0=bdma_cw_width_max+1;	{ steps between centers }
	mr=mx0*my0(UU);
	ay0=dm(bdma_cwfilter_width);	{ add width }
	ar=mr0+ay0; ar=ar+ay0;
	sr=LSHIFT ar by 7(LO);		{ 0x100 bytes per filter step}

	ay0=0x3fff;			{ mask low order 14 bits }
	ar=sr0 AND ay0;			{ this is bdma address within page }
	dm(BDMA_Ext_Address)=ar;
	sr=LSHIFT sr0 by -6 (LO);	{ drop low order 14 bits }
	ay0=0x0800;			{ base page >> 2 }
	ar=sr0+ay0;			{ this is bdma page }
	ay0=0xff00;			{ mask off leftover address bits }
	ar=ar AND ay0;
	ay0=1;				{ MCR=0, BDIR=read, BTYPE=16 bit DM }
	ar=ar OR ay0;
	dm(BDMA_Control)=ar;

	ax0=^temp_coeffs;		{ point to destination in dm }
	dm(BDMA_Int_Address)=ax0;

	ax0=FILTER_SIZE;		{ set up transfer length and start transfer }
	dm(BDMA_Word_Count)=ax0;

	ar=dm(bdma_cwfilter_center);	{ update cw_fil info while transfer taking place }
	sr=LSHIFT ar by 4(LO);		{ shift to upper nibble of lower byte }
	ay0=dm(bdma_cwfilter_width);
	ax0=0x0F;
	af=ax0 AND ay0;
	ar=sr0 OR af;
	ay0=0xFF;
	ar=ar AND ay0;
	dm(cw_fil)=ar;
	dm(filter_info)=ar;

	ar=0; dm(SSB_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(CW_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x01;			{ mode = cw }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;

check_cw_bdma:				{ wait for transfer to complete }
	ax0=dm(BDMA_Word_Count);
	ar=PASS ax0;
	if NE jump check_cw_bdma;

	dis ints;

{ save index registers and point to coefficients }
	ax0=i7; dm(save_i7)=ax0; i7=^CW_coeffs;

	jump bdma_coeff_update;

{----------------------------------------------------------------------------}
{             	BDMA DATA filter control commands  			     }
{----------------------------------------------------------------------------}

do_RTTY_high:
	ar=0x02;			{ width = 200 Hz }
	dm(bdma_datafilter_width)=ar;
	ar=0x0B;			{ center = 2100 Hz }
	dm(bdma_datafilter_center)=ar;
	jump do_RTTY_1;

do_RTTY_med:
	ar=0x03;			{ width = 300 Hz }
	dm(bdma_datafilter_width)=ar;
	ar=0x07;			{ center = 1700 Hz }
	dm(bdma_datafilter_center)=ar;
	jump do_RTTY_1;

do_RTTY_low:
	ar=0x01;			{ width = 100 Hz }
	dm(bdma_datafilter_width)=ar;
	ar=0x00;			{ center = 1000 Hz }
	dm(bdma_datafilter_center)=ar;
do_RTTY_1:
	ar=0;
	dm(SSB_switch)=ar;
	dm(CW_switch)=ar;
	dm(SOFT_switch)=ar;
	ar=1;
	dm(DATA_switch)=ar;
	jump BDMA_data_setup;		{ load new coeffs }


BDMA_data_higher_freq:
	ax0=dm(bdma_datafilter_center);	{ get current center freq }
	ay0=bdma_data_center_max;	{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_datafilter_center)=ar;
	jump BDMA_data_setup;		{ and load new coeffs }

BDMA_data_lower_freq:
	ax0=dm(bdma_datafilter_center);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_datafilter_center)=ar;
	jump BDMA_data_setup;		{ and load new coeffs }

BDMA_data_wider:
	ax0=dm(bdma_datafilter_width);	{ get current center freq }
	ay0=bdma_data_width_max;	{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_datafilter_width)=ar;
	jump BDMA_data_setup;		{ and load new coeffs }

BDMA_data_narrower:
	ax0=dm(bdma_datafilter_width);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_datafilter_width)=ar;
	jump BDMA_data_setup;		{ and load new coeffs }

BDMA_data_setup:
	mx0=dm(bdma_datafilter_center);
	my0=bdma_data_width_max+1;	{ steps between centers }
	mr=mx0*my0(UU);
	ay0=dm(bdma_datafilter_width);	{ add width }
	ar=mr0+ay0; ar=ar+ay0;
	sr=LSHIFT ar by 7(LO);		{ 0x100 bytes per filter step}

	ay0=0x3fff;			{ mask low order 14 bits }
	ar=sr0 AND ay0;
	dm(BDMA_Ext_Address)=ar;

	sr=LSHIFT sr0 by -6 (LO);	{ drop low order 14 bits }
	ay0=0x0c00;			{ base page >> 2 }
	ar=sr0+ay0;			{ this is bdma page }
	ay0=0xff00;			{ mask off leftover address bits }
	ar=ar AND ay0;
	ay0=1;				{ MCR=0, BDIR=read, BTYPE=16 bit DM }
	ar=ar OR ay0;
	dm(BDMA_Control)=ar;

	ax0=^temp_coeffs;		{ point to destination in dm }
	dm(BDMA_Int_Address)=ax0;

	ax0=FILTER_SIZE;		{ set up transfer length and start transfer }
	dm(BDMA_Word_Count)=ax0;

	ar=dm(bdma_datafilter_center);	{ update data_fil info while transfer taking place }
	sr=LSHIFT ar by 4(LO);		{ shift to upper nibble of lower byte }
	ay0=dm(bdma_datafilter_width);
	ax0=0x0F;
	af=ax0 AND ay0;
	ar=sr0 OR af;
	ay0=0xFF;
	ar=ar AND ay0;
	dm(rtty_fil)=ar;
	dm(filter_info)=ar;

	ar=0; dm(CW_switch)=ar; dm(SSB_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(DATA_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x03;			{ mode = rtty }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;

check_data_bdma:			{ wait for transfer to complete }
	ax0=dm(BDMA_Word_Count);
	ar=PASS ax0;
	if NE jump check_data_bdma;

	dis ints;

{ save index registers and point to coefficients }
	ax0=i7; dm(save_i7)=ax0; i7=^DATA_coeffs;

	jump bdma_coeff_update;

{----------------------------------------------------------------------------}
{             	BDMA SSB filter control commands  			     }
{----------------------------------------------------------------------------}

do_SSB_notch_denoise:
	ar=0xFFFF;			{ value for ON }
	dm(Autonotch_switch)=ar;	{ turn on autonotch }
	dm(Denoise_switch)=ar;		{ turn on DeNoiser }
	ax0=0x08;			{ set denoise bit }
	ay0=dm(status_flag);
	ar=ax0 OR ay0;
	dm(status_flag)=ar;
	jump do_SSB_2;

do_SSB_notch:
	ar=0xFFFF;			{ value for ON }
	dm(Autonotch_switch)=ar;	{ turn on autonotch }
	ax0=0x04;			{ set autonotch bit }
	ay0=dm(status_flag);
	ar=ax0 OR ay0;
	dm(status_flag)=ar;
	jump do_SSB_1;

do_SSB_only:
	ar=0;				{ value for OFF }
	dm(Autonotch_switch)=ar;	{ turn off AutoNotch }
	ax0=0xFB;			{ clear autonotch bit }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	dm(status_flag)=ar;

do_SSB_1:
	ar=0;
	dm(Denoise_switch)=ar;		{ turn off DeNoiser }
	ax0=0xF7;			{ clear denoiser bit }
	ay0=dm(status_flag);
	ar=ax0 AND ay0;
	dm(status_flag)=ar;
do_SSB_2:
	ar=0;
	dm(CW_switch)=ar;		{ turn off CW mode }
	dm(DATA_switch)=ar;		{ turn off DATA mode }
	dm(SOFT_switch)=ar;
	ar=1;
	dm(SSB_switch)=ar;
	ar=0x0A;			{ upper corner 2500 Hz }
	dm(bdma_ssbfilter_high)=ar;
	ar=0x3;
	dm(bdma_ssbfilter_low)=ar;	{ lower corner 200 Hz }
	jump BDMA_ssb_setup;		{ load new coeffs }

BDMA_ssb_higher_freq:
	ax0=dm(bdma_ssbfilter_high);	{ get current center freq }
	ay0=bdma_ssb_upper_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_ssbfilter_high)=ar;
	jump BDMA_ssb_setup;		{ and load new coeffs }

BDMA_ssb_lower_freq:
	ax0=dm(bdma_ssbfilter_high);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_ssbfilter_high)=ar;
	jump BDMA_ssb_setup;		{ and load new coeffs }

BDMA_ssb_narrower:
	ax0=dm(bdma_ssbfilter_low);	{ get current center freq }
	ay0=bdma_ssb_lower_max;		{ ? already at max }
	ar=ay0-ax0;
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0+1;			{ no - increment value }
	dm(bdma_ssbfilter_low)=ar;
	jump BDMA_ssb_setup;		{ and load new coeffs }

BDMA_ssb_wider:
	ax0=dm(bdma_ssbfilter_low);	{ get current center freq }
	ar=PASS ax0;			{ ? already at min }
	if EQ jump command_ok;		{ yes - exit }
	ar=ax0-1;			{ no - decrement value }
	dm(bdma_ssbfilter_low)=ar;
	jump BDMA_ssb_setup;		{ and load new coeffs }

BDMA_ssb_setup:
	mx0=dm(bdma_ssbfilter_high);
	my0=bdma_ssb_lower_max+1;	{ steps between centers }
	mr=mx0*my0(UU);
	ay0=dm(bdma_ssbfilter_low);	{ add width }
	ar=mr0+ay0; ar=ar+ay0;
	sr=LSHIFT ar by 7(LO);		{ 0x100 bytes per filter step}

	ay0=0x3fff;			{ mask low order 14 bits }
	ar=sr0 AND ay0;			{ this is bdma address within page }
	dm(BDMA_Ext_Address)=ar;

	sr=LSHIFT sr0 by -6 (LO);	{ drop low order 14 bits }
	ay0=0x1000;			{ base page >> 2 }
	ar=sr0+ay0;			{ this is bdma page }
	ay0=0xff00;			{ mask off leftover address bits }
	ar=ar AND ay0;
	ay0=1;				{ MCR=0, BDIR=read, BTYPE=16 bit DM }
	ar=ar OR ay0;
	dm(BDMA_Control)=ar;

	ax0=^temp_coeffs;		{ point to destination in dm }
	dm(BDMA_Int_Address)=ax0;

	ax0=FILTER_SIZE;		{ set up transfer length and start transfer }
	dm(BDMA_Word_Count)=ax0;

	ar=dm(bdma_ssbfilter_high);	{ update ssb_fil info while transfer taking place }
	sr=LSHIFT ar by 4(LO);		{ shift to upper nibble of lower byte }
	ay0=dm(bdma_ssbfilter_low);
	ax0=0x0F;
	af=ax0 AND ay0;
	ar=sr0 OR af;
	ay0=0xFF;
	ar=ar AND ay0;
	dm(ssb_fil)=ar;
	dm(filter_info)=ar;

	ar=0; dm(CW_switch)=ar; dm(DATA_switch)=ar; dm(SOFT_switch)=ar;
	ar=1; dm(SSB_switch)=ar;
	ar=dm(status_flag);
	ay0=0xFC; ay1=0x02;			{ mode = ssb }
	ar=ar AND ay0; ar=ar OR ay1;
	dm(status_flag)=ar;


check_ssb_bdma:				{ wait for transfer to complete }
	ax0=dm(BDMA_Word_Count);
	ar=PASS ax0;
	if NE jump check_ssb_bdma;

	dis ints;			{ save index registers and point to coefficients }
	ax0=i7; dm(save_i7)=ax0; i7=^SSB_coeffs;

bdma_coeff_update:
	ax0=i0; dm(save_i0)=ax0; i0=^temp_coeffs;
	ax0=l0; dm(save_l0)=ax0; l0=0;
	ax0=m0; dm(save_m0)=ax0; m0=1;	

	ax0=l7; dm(save_l7)=ax0; l7=0;
	ax0=m7; dm(save_m7)=ax0; m7=1;

{ move coefficients from dm to pm }
	CNTR=FILTER_SIZE;
	DO coeff_update UNTIL CE;
		ax0=dm(i0,m0);
coeff_update:	pm(i7,m7)=ax0;


{ restore index registers }
	ax0=dm(save_i0);	i0=ax0;	{ restore index registers }
	ax0=dm(save_l0);	l0=ax0;
	ax0=dm(save_m0);	m0=ax0;
	ax0=dm(save_i7);	i7=ax0;
	ax0=dm(save_l7);	l7=ax0;
	ax0=dm(save_m7);	m7=ax0;

	ena ints;

	jump command_ok;

{----------------------------------------------------------------------------}
{      Subroutine used for Codec initialization, Rev C Module	   	     }
{									     }
{ This subroutine outputs the 16-bit value in ax0 to the		     }
{ TLV320AIC23 codec control port, set up as a SPI port.			     }
{									     }
{ It works with the ADSP-218xN running at maximum speed (80 MIPs).	     }
{									     }
{ The data is shifted out as a 16-bit value, MSB first, per		     }
{ the AIC data sheet.							     }
{									     }
{ Registers altered:							     }
{	ar, sr, ax1, ay1, af, status flag				     }
{									     }
{ Written:  10 Feb 2003 L Johnson (Rev B)				     }
{ Modified: 21 Feb 2003 L Johnson (Rev C)				     }
{									     }
{----------------------------------------------------------------------------}
CODEC_CMD:
	ax1=16;			{ bit count }
	ay1=SPICS+SPIDAT;	{ mask to set SPICLK low }
	sr0=ax0; 		{ data to shift out }
	ar=SPICS;		{ SPICS high, SPICLK low, SPIDAT low }
	io(CODEC)=ar;		{ initial conditions }
	ar=ar and SPIDAT;	{ SPICS low }
	nop; nop;		{ delay for AIC }
	io(CODEC)=ar;		{ now ready to send cmd }

CODEC1:
	ar=0;			{ set data bit = 0 }
	af=sr0 AND 0x8000;	{ if MSB is set.. }
	if ne ar=ar or SPIDAT;	{ ..data bit = 1 }
	io(CODEC)=ar;		{ output data bit }
	nop; nop;		{ ..delay for AIC }
	ar=ar or SPICLK;	{ ..pulse SPLICLK high }
	io(CODEC)=ar;
	nop; nop;		{ ..delay for AIC }
	ar=ar and SPIDAT;	{ ..bring SPICLK low again }
	io(CODEC)=ar;
	ar=ax1-1;		{ decrement bit counter }
	if eq jump CODEC2;      { exit if last bit sent, else..}
	ax1=ar;			{ ..save bit counter, }
	sr=lshift sr0 by 1(LO);	{ ..promote next bit to MSB, }
	jump CODEC1;		{ ..and loop until done }

CODEC2:				{ clean up and exit }
	ar=ar or SPICS;		{ SPICS high to latch cmd, }
	nop; nop;		{ ..delay for AIC }
	io(CODEC)=ar;		{ ..output CS bit }

	rts;			{ ..and return to caller }


{------------------ compute gain ---------------------}
{                                                     }
{ expects value to evaluate in ax1                    }
{ very crude, I admit, but I was in a hurry!          }
{ returns shift in ax0, multiplier in ay0             }
{                                                     }
{-----------------------------------------------------}
figger_gain:
	sr0=ax1;
	sr=lshift sr0 by -4 (LO);	{ extract upper nibble as shift }
	ay0=0x03;		{ limit range }
	ar=sr0 AND ay0;
	ax0=ar;			{ shift in ax0 }

	ay0=0x0f;
	ar=ax1 AND ay0;		{ limit to low nibble }
		
	ay0=^gain_small_step;
	ar=ar+ay0;	{ pointer to small gain value }
	
	dis ints;	{ save index register }
	ay0=i7;
	dm(save_i7)=ay0;
	i7=ar;
	ay0=pm(i7,m7);	{ return small gain value }

	ar=dm(save_i7);	{ restore index register }
	i7=ar;
	ena ints;

	rts;		{ done! }

.var/pm     gain_small_step[16];            { Coefficients for the filter   }
.init24 gain_small_step:

	0x43F400,	{ gain = -5.5 dB }
	0x47FA00,	{ gain = -5   dB }
	0x4C3E00,	{ gain = -4.5 dB }
	0x50C300,	{ gain = -4   dB }
	0x558C00,	{ gain = -3.5 dB }
	0x5A9D00,	{ gain = -3   dB }
	0x5FFC00,	{ gain = -2.5 dB }
	0x65AC00,	{ gain = -2   dB }
	0x6BB200,	{ gain = -1.5 dB }
	0x721400,	{ gain = -1   dB }
	0x78D600,	{ gain = -0.5 dB }
	0x7FFF00,	{ gain =  0   dB }
	0x7FFF00,	{ gain =  0   dB }
	0x7FFF00,	{ gain =  0   dB }
	0x7FFF00,	{ gain =  0   dB }
	0x7FFF00;	{ gain =  0   dB }

{----------------------------------------------------------------------------}
{   A high to low transition on flag_in signifies the start bit; it also     }
{   triggers IRQ1 ISR which then turns on timer if the timer is off. This is }
{   at the most 1/3 bit period too late but we should still catch the byte.  }
{----------------------------------------------------------------------------}
irq1isr:
{***}	reset FL0;	{ diagnostic }
        pop sts;
        ena timer;              { start timer now }

        rts;                    { note rts }
{----------------------------------------------------------------------------}

{----------------------------------------------------------------------------}
{      BDMA interrupt handler						     }
{      This signals the transfer is complete and the data must be moved      }
{      from data memory to program memory.                                   }
{----------------------------------------------------------------------------}
bdma_irq:
	ifc=b#0000000000001000;		{ clear bdma interrupt }

        rti;
{----------------------------------------------------------------------------}

.endmod;
