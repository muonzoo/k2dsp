.module/RAM       audio_isr;

{************************************************************************
 *
 * Interrupt Servcice Routine Module
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
 * This module's routines are based extensively on the work of Johan
 * Forrer, KC7WW, Monroe OR, and borrow heavily from Johan's ASP article
 * which appeared in QEX for September, 1996.
 *
 * A great debt is also owed to the work of Bob Larkin in the DSP-10 project
 * from QST, September - November, 1999.
 *
 * Changes include:
 *
 * mode flags to determine path
 * changes in decimator
 * addition of controllable gain blocks
 * use of both channels of the CODEC to support K2 differential audio 
 *
 ****************************************************************************}

.ENTRY input_isr, input_init;	{ entry points to this module }

{ EXTERNALs are program or data locations used by this module, but created
  in other modules }

.EXTERNAL  LMS;	
.EXTERNAL  sigout_R, sigout_L, rx_buf, tx_buf;

{ GLOBALs are program or data locations created by this module, but made
  available to other modules }

.GLOBAL CW_switch, SSB_switch, DATA_switch, CW_coeffs, SSB_coeffs, DATA_coeffs;
.GLOBAL cwfil_large_gain;
.GLOBAL cwfil_small_gain;
.GLOBAL ssbfil_large_gain;
.GLOBAL ssbfil_small_gain;
.GLOBAL datafil_large_gain;
.GLOBAL datafil_small_gain;


.const FILTER_SIZE=128;         { CW and SSB and DATA }
.const DEC2_SIZE=48;		{ by 2 decimation/interpolation filter }

{ Data locations required by this module that are not EXTERNAL.  GLOBALs
  must be declared here, too. }

.var/dm/circ            dl[FILTER_SIZE];        { delay line for filter }
.var/dm/circ		d2_input[DEC2_SIZE];	{ input by-2 decimator }
.var/dm/circ		d2_output[DEC2_SIZE];	{ output by-2 interpolator }
.var/dm                 filter_dl_ptr;          { Filter delay line pointer }
.var/dm 		d2_in_ptr;		{ by-2 decimator variables }
.var/dm 		d2_out_ptr;
.var/dm			d2_cntr;
.var/dm			CW_switch;		{ CW filter selection }
.var/dm			SSB_switch;		{ SSB filter selection }
.var/dm			DATA_switch;		{ RTTY filter selection }
.var/dm			cwfil_large_gain;
.var/dm			cwfil_small_gain;
.var/dm			ssbfil_large_gain;
.var/dm			ssbfil_small_gain;
.var/dm			datafil_large_gain;
.var/dm			datafil_small_gain;

{ The following data files are in PROGRAM memory.  They are filter coefficients.
  They will be initialized here, but are reloaded by the BDMA routines in the
  ASP module. }

.var/pm		CW_coeffs[FILTER_SIZE];	{ Coefficients for the CW filter }
.init24 CW_coeffs:
.include <cw8060.dat>; { this is a bandpass filter, center 800 Hz, width 600 Hz }

.var/pm		SSB_coeffs[FILTER_SIZE];	{ Coefficients for the SSB filter }
.init24 SSB_coeffs:
.include <sb2502.dat>; { this is a bandpass filter, upper 2500 Hz, lower 200 Hz }

.var/pm		DATA_coeffs[FILTER_SIZE];	{ Coefficients for the DATA filter }
.init24 DATA_coeffs:
.include <da2202.dat>; { this is a bandpass filter, center 2200 Hz, width 200 Hz }

.var/pm		dec2_coef[DEC2_SIZE];	{ Coefficients for the dec/interp filter }
.init24 dec2_coef:
.include <dec48.dat>;  { this is a low pass filter }

{----------------------------------------------------------------------------}
{ Code initialization call			                             }
{----------------------------------------------------------------------------}
input_init:

{ init_filter() is used to set up the DAG registers used by the filter }
{ following a reset.                                                   }
	m0=1;
	m4=1;

	i0=^dl;
	dm(filter_dl_ptr)=i0;
	ar=0;				{ disable CW, SSB and DATA filters }
	dm(CW_switch)=ar;
	dm(SSB_switch)=ar;
	dm(DATA_switch)=ar;

	i0=^d2_input;
	dm(d2_in_ptr)=i0;		{ by-2 decimator variables }

	i0=^d2_output;
	dm(d2_out_ptr)=i0;

	ar=0;
	dm(d2_cntr)=ar;

	rts;

{----------------------------------------------------------------------------}
{   SPORT0 receive interrupt handler					     }
{									     }
{   This is where the action is.  This routine is entered 16,000 times per   }
{   second.  Input damples are dealt with, and all audio processing is 	     }
{   invoked from this routine.						     }
{									     }
{----------------------------------------------------------------------------}
input_isr:
        ena sec_reg;                	{ use shadow register bank           }

	mr1=dm(sigout_R);		{ pick up processed out/input signal }
        dm (tx_buf + 0) = mr1;      	{ filtered output to SPORT (to spkr) }
	mr1=dm(sigout_L);
        dm (tx_buf + 1) = mr1;

        ax0 = dm (rx_buf + 0);     { get new sample from SPORT0 (from codec) }

{ *** k2 is differential }

	ay0= dm(rx_buf+1 );	{ get other phase }
	ar=ax0-ay0;		{ desired signal is difference }
	ax0=ar;			{ result in ax0 }

{ *** end differential }

{----------------------------------------------------------------------------}
{ 			  Decimate by 2 for all modes			     }
{									     }
{  Decimate to reduce effective sampling rate.  Allows FIR filters to be     }
{  shorter than otherwise.  Program structure allows different decimation    }
{  rates to be used for other modes.  Original CW routines used decimation   }
{  by 6, for example.							     }
{									     }
{----------------------------------------------------------------------------}
do_by2:
	call by2_dec;		 { takes ax0, result in ax0 }

	ay0=dm(d2_cntr);   	 { use only every 2nd point }
	ar=ay0+1;
	ay0=1;
	ar=ar AND ay0;
	dm(d2_cntr)=ar;
	if NE jump send_Zeroes;

	ar=dm(SSB_switch); 	{ SSB filtering ? }
	none=PASS ar;
	if EQ jump check_data;

	call SSB_filter;	{ yes - do SSB filter }
	jump do_Interp;

check_data:
	ar=dm(DATA_switch);	{ DATA filtering? }
	none=PASS ar;
	if EQ jump do_cw;	{ no - must be CW filter }

	call DATA_filter;	{ yes - do DATA filter }
	jump do_Interp;

do_cw:
	call CW_filter;
	jump do_Interp;

send_Zeroes:
	ax0=0;			{ otherwise send zeroes through filter       }
do_Interp:
	call by2_interpol;	{ send filtered data through the interpolator}
	call LMS;	 

	dm(sigout_R)=ax0;

{ *** k2 is differential }

	ar= -ax0;
	ax0=ar;

{ *** end differential }

	dm(sigout_L)=ax0;
	dis sec_reg;
	rti;

{----------------------------------------------------------------------------}
{   CW Filter								     }
{									     }
{   This is a 128-tap FIR filter.					     }
{									     }
{----------------------------------------------------------------------------}						     
CW_filter:
	i0=dm(filter_dl_ptr);
	l0=FILTER_SIZE;
	l4=0;				{ NOT CIRC !! }
	ar=dm(CW_switch);
	i4=^CW_coeffs;                    { load CW filter coeffs }

CW_filt_1:
	m0=1;
	m4=1;
	dm(i0,m0)=ax0;			{ place new data in buffer }
	cntr=FILTER_SIZE-1;             { cntr=filtersize - 1 }
	mr=0, mx0=dm(i0,m0), my0=pm(i4,m4);
	do CW_fir_loop until ce;
CW_fir_loop:  mr=mr+mx0*my0(ss), mx0=dm(i0,m0), my0=pm(i4,m4);

	mr=mr+mx0*my0(rnd);
	if mv sat mr;
	ax0=mr1;			{ unadjusted result }

{------------- adjust cw filter gain ----------------------------------------}

	se=dm(cwfil_large_gain);	{ do shift for large steps }
	si=ax0;
	sr=ashift si (HI);
	my0=dm(cwfil_small_gain);	{ do multiply for fine steps }
	mr=sr1 * my0 (ss);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	ar=mr1;				{ fetch result of attenuator }
	ax0=ar;

	dm(filter_dl_ptr)=i0;		{ save pointer }
	rts;

{----------------------------------------------------------------------------}
{   SSB Filter								     }
{									     }
{   This is a 128-tap FIR filter.					     }
{									     }
{----------------------------------------------------------------------------}		
SSB_filter:
	i0=dm(filter_dl_ptr);
	l0=FILTER_SIZE;
	l4=0;				{ NOT CIRC !! }
	i4=^SSB_coeffs;                   { load SSB filter coeffs }

SSB_filt_1:
	m0=1;
	m4=1;
	dm(i0,m0)=ax0;			{ place new data in buffer }
	cntr=FILTER_SIZE-1;             { cntr=filtersize - 1 }
	mr=0, mx0=dm(i0,m0), my0=pm(i4,m4);
	do SSB_fir_loop until ce;
SSB_fir_loop:  mr=mr+mx0*my0(ss), mx0=dm(i0,m0), my0=pm(i4,m4);

	mr=mr+mx0*my0(rnd);
	if mv sat mr;
	ax0=mr1;			{ unadjusted result }

{------------- adjust ssb filter gain ---------------------------------------}

	se=dm(ssbfil_large_gain);	{ do shift for large steps }
	si=ax0;
	sr=ashift si (HI);
	my0=dm(ssbfil_small_gain);	{ do multiply for fine steps }
	mr=sr1 * my0 (ss);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	ar=mr1;				{ fetch result of attenuator }
	ax0=ar;

	dm(filter_dl_ptr)=i0;		{ save pointer }
	rts;

{----------------------------------------------------------------------------}
{   DATA Filter								     }
{									     }
{   This is a 128-tap FIR filter.					     }
{									     }
{----------------------------------------------------------------------------}		
DATA_filter:
	i0=dm(filter_dl_ptr);
	l0=FILTER_SIZE;
	l4=0;				{ NOT CIRC !! }
	i4=^DATA_coeffs;                    { select RTTY filter coeffs }

DATA_filt_1:
	m0=1;
	m4=1;
	dm(i0,m0)=ax0;			{ place new data in buffer }
	cntr=FILTER_SIZE-1;             { cntr=filtersize - 1 }
	mr=0, mx0=dm(i0,m0), my0=pm(i4,m4);
	do DATA_fir_loop until ce;
DATA_fir_loop:  mr=mr+mx0*my0(ss), mx0=dm(i0,m0), my0=pm(i4,m4);

	mr=mr+mx0*my0(rnd);
	if mv sat mr;
	ax0=mr1;			{ unadjusted result }

{------------- adjust data filter gain --------------------------------------}

	se=dm(datafil_large_gain);	{ do shift for large steps }
	si=ax0;
	sr=ashift si (HI);
	my0=dm(datafil_small_gain);	{ do multiply for fine steps }
	mr=sr1 * my0 (ss);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	ar=mr1;				{ fetch result of attenuator }
	ax0=ar;

	dm(filter_dl_ptr)=i0;		{ save pointer }
	rts;

{----------------------------------------------------------------------------}
{   Decimator Filter							     }
{									     }
{   This is a 48-tap FIR filter.					     }
{									     }
{----------------------------------------------------------------------------}		
by2_dec:
	i0=dm(d2_in_ptr);
	l0=DEC2_SIZE;
	i4=^dec2_coef;                   { point to decimator coefficients }
	l4=0;				{ NOT CIRC !! }
	m0=1;
	m4=1;
	dm(i0,m0)=ax0;			{ place new data in buffer }
	cntr=DEC2_SIZE-1;               { cntr=DEC2_SIZE - 1 }
	mr=0, mx0=dm(i0,m0), my0=pm(i4,m4);
	do by2_dec_fir_loop until ce;
by2_dec_fir_loop:  mr=mr+mx0*my0(ss), mx0=dm(i0,m0), my0=pm(i4,m4);

	mr=mr+mx0*my0(SS);
	my1=1;				{ correct for loss of gain }
	mx1=mr1;
	mr=mr0*my1(UU);
	mr0=mr1;
	mr1=mr2;
	mr=mr+mx1*my1(SU);	
	sr=lshift mr0 by -1 (LO);
	sr=sr or ashift mr1 by -1 (HI);	
	ax0=sr0; 			{ result }
	dm(d2_in_ptr)=i0;		{ save pointer }
	rts;

{----------------------------------------------------------------------------}
{   Interpolator Filter							     }
{									     }
{   This is a 48-tap FIR filter.					     }
{									     }
{----------------------------------------------------------------------------}		
by2_interpol:
	i0=dm(d2_out_ptr); 			{ select one of these }
	l0=DEC2_SIZE;
	i4=^dec2_coef;                   { point to coefficients }
	l4=0;				{ NOT CIRC !! }
	m0=1;
	m4=1;
	dm(i0,m0)=ax0;			{ place new data in buffer }
	cntr=DEC2_SIZE-1;               { cntr=DEC2_SIZE - 1 }
	mr=0, mx0=dm(i0,m0), my0=pm(i4,m4);
	do by2_interp_fir_loop until ce;
by2_interp_fir_loop:  mr=mr+mx0*my0(ss), mx0=dm(i0,m0), my0=pm(i4,m4);

	mr=mr+mx0*my0(SS);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	my1=2;				{ correct for loss of gain }
	mx1=mr1;
	mr=mr0*my1(UU);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	mr0=mr1;
	mr1=mr2;
	mr=mr+mx1*my1(SU);
{***}
	if mv sat mr;			{ limit amplitude }
{***}
	sr=lshift mr0 by -1 (LO);
	sr=sr or ashift mr1 by -1 (HI);	
	ax0=sr0; 			{ result }
	dm(d2_out_ptr)=i0;		{ save pointer }
	rts;

{----------------------------------------------------------------------------}
.ENDMOD;
