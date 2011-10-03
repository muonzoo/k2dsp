.MODULE LMS_main;

{************************************************************************
 *
 * LMS Denoise and Auto-notch Module
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
 * These Noise-Reduction and Auto-Notch LMS routines are based extensively
 * on the work of Johan Forrer, KC7WW, Monroe OR, and borrow heavily from
 * Johan's ASP article which appeared in QEX for September, 1996.
 *
 * A great debt is also owed to the work of Bob Larkin in the DSP-10 project
 * from QST, September - November, 1999.
 *
 * Both of these works in turn are based on work of:
 *
 * Dave Hershberger, W9GR, and Dr. S. Reyer, WA9VNJ, from QST and QEX
 * September, 1992.
 * 
 * Routines from Analog Devices Applications Using the ADSP 2100 Family,
 * Volumes 1 and 2.
 *
 * Changes include:
 *
 * addition of multiple length choices in the denoiser
 * gain balance routines based on mode flags
 *
 * 17 July 2003
 * added LMS_init_all entry and changed LMS_Inti to not re-init the beta
 *   and decay values
 *
 ****************************************************************************}

 { Constants for this module }

.const taps=91;			{ initial delay line length                  }
.const aut_taps=64;		{ AutoNotch delay line length                }
.const delay_taps=127;		{ AutoNotch delay line                       }
.const num_fine_steps=12;	{ coefficients for small gain steps          }

 { Entry Points for this module }

.ENTRY LMS_init, Denoise_init, LMS, LMS_init_all;

 { Data from other modules needed by this module }

.EXTERNAL DATA_switch, SSB_switch, CW_switch; { mode state flags             }

 { Data from this module available to other modules }

.GLOBAL denoise_taps;		{ length of denoise filter                   }
.GLOBAL denoise_tapsm1;		{ length of denoise filter-1                 }
.GLOBAL beta; 			{ beta factor for denoising                  }
.GLOBAL beta_aut;  		{ beta factor for autonotch                  }
.GLOBAL decay;			{ decay factor for denoiser                  }
.GLOBAL decay_aut;		{ decay factor for autonotch                 }
.GLOBAL Denoise_switch;		{ denoiser in/out                            }
.GLOBAL Autonotch_switch;	{ autonotch in/out                           }
.GLOBAL unprocessed_sample;	{ raw data input                             }
.GLOBAL denoised_sample;	{ raw after denoising                        }
.GLOBAL notched_sample;		{ raw or denoised after notching             }
.GLOBAL denoiser_large_gain;	{ ~6 dB gain steps for denoiser gain         }
.GLOBAL denoiser_small_gain;	{ ~0.5 dB gain steps for denoiser gain       }
.GLOBAL autonotch_large_gain;	{ ~6 dB gain steps for autonotch gain        }
.GLOBAL autonotch_small_gain;	{ ~0.5 dB gain steps for autonotch gain      }

{----------------------------------------------------------------------------}
{                     LMS working storage		                     }
{----------------------------------------------------------------------------}
.var/dm/ram output;		{ FIR output                           	     }
.var/dm/ram sample;		{ input sample                         	     }
.var/dm/ram denoise_taps;	{ length of denoise filter                   }
.var/dm/ram denoise_tapsm1;	{ length of denoise filter-1                 }
.var/dm/ram beta;		{ beta factor for denoising                  }
.var/dm/ram beta_aut;		{ beta factor for autonotch                  }
.var/dm/ram e_beta;		{ denoiser beta error feedback term	     }
.var/dm/ram e_beta1;		{ as above, for autonotching                 }
.var/dm/ram decay ;		{ decay factor for denoiser                  }
.var/dm/ram decay_aut;		{ decay factor for autonotch                 }
.var/dm/ram r_ar1 ;		{ pointer into coeff. array                  }
.var/dm/ram gain  ;     	{ audio output gain setting                  }
.var/dm/ram Error ;		{ error term temp storage for LMS algorithm  }
.var/dm/ram Autonotch_switch;   { autonotch in/out                           }
.var/dm/ram Denoise_switch;     { denoiser in/out                            }
.var/dm/ram r_ar2;		{ pointer into coeff. array           	     }
.var/dm/ram denoiser_large_gain;  { ~6 dB gain steps for denoiser gain       }
.var/dm/ram denoiser_small_gain;  { ~0.5 dB gain steps for denoiser gain     }
.var/dm/ram autonotch_large_gain; { ~6 dB gain steps for autonotch gain      }
.var/dm/ram autonotch_small_gain; { ~0.5 dB gain steps for autonotch gain    }

{ Index Register Storage }

.var/dm/ram i1_ptr;
.var/dm/ram i2_ptr;
.var/dm/ram i3_ptr;

.var/dm/ram save_i2;
.var/dm/ram save_i3;

{----------------------------------------------------------------------------}
{                     LMS results storage		                     }
{----------------------------------------------------------------------------}
.var/dm/ram unprocessed_sample;	{ input from calling routine 		     }
.var/dm/ram denoised_sample;	{ denoised 				     }
.var/dm/ram notched_sample;	{ notched 				     }	

{----------------------------------------------------------------------------}
{                     LMS circular buffer storage		             }
{----------------------------------------------------------------------------}
.var/dm/ram/circ data[aut_taps];	{ circular data delay line           }
.var/dm/ram/circ data1[taps];		{ second delay line                  }
.var/dm/ram/circ D[delay_taps];		{ input delay line                   }

{----------------------------------------------------------------------------}
{               STORAGE FOR ADAPTIVE COEFFICIENTS                            }
{----------------------------------------------------------------------------}
.var/pm/circ coeff[aut_taps];		{ autonotch			     }
.var/pm/circ coeff1[taps];		{ denoiser			     }

{------------------- Initialization------------------------------------------}
LMS_init_all:
	ax0=0x0A3D;		{ 0.080 Beta                                 }
	dm(beta_aut)=ax0;
	ax0=0x7EB8;	  	{ 0.99  Decay                                }
	dm(decay_aut)=ax0;

{ Beta values that work  with 19kHz sample rate 4C00 - 1000                  }
{ values closer to 1000 work  best for denoiser                              }

	ax0=0x31EB;		{ 0.039 Beta                                 }
	dm(beta) =ax0;
	ax0=0x747A;	  	{ 0.91  Decay                                }
	dm(decay)=ax0;

LMS_init:
	ax0=0; 				{ disable function }
	dm(Autonotch_switch)=ax0;

{	ax0=0x0A3D;	}	{ 0.080 Beta                                 }
{	dm(beta_aut)=ax0;	}
{	ax0=0x7EB8;	  }	{ 0.99  Decay                                }
{	dm(decay_aut)=ax0;	}

	i0=^coeff;		{ Denotching coefficient pointer }
	dm(r_ar2)=i0;

        i0=^data; 		{ I2 }
	dm(i2_ptr)=i0;

	i0=^D; 			{ I3 }
	dm(i3_ptr)=i0;

{ fall through to denoise initialization }

Denoise_init:

{ Beta values that work  with 19kHz sample rate 4C00 - 1000                  }
{ values closer to 1000 work  best for denoiser                              }

{	ax0=0x31EB;	}	{ 0.039 Beta                                 }
{	dm(beta) =ax0;	}
{	ax0=0x747A;	} 	{ 0.91  Decay                                }
{	dm(decay)=ax0;	}

	i0=^coeff1;		{ Denoiser coefficient pointer }
	dm(r_ar1)=I0;

        i0=^data1;  		{ I1 }
	dm(i1_ptr)=i0;

	ax0=taps;		{ set initial denoise length }
	dm(denoise_taps)=ax0;
	ar=ax0-1;
	dm(denoise_tapsm1)=ar;

	ax0=0; 			{ disable function }
	dm(Denoise_switch)=ax0;

	rts;

{----------------------------------------------------------------------------}
{                         LMS processing				     }
{        Input sample is in ax0, result is returned in ax0	    	     }
{----------------------------------------------------------------------------}
LMS:    	
	dm(sample)=ax0;			{ save input sample }
	dm(unprocessed_sample)=ax0;	{ save here too! }

	dis m_mode;			{ select 2's complement mode }

{ ------------- Autonotch - always run it -----------------------------------}

Autonotch:
	dm(save_i2)=i2;			{ save i2 and i3 }
	dm(save_i3)=i3;

autonotch_it:
	i2=dm(i2_ptr);			{ set up index and length registers }
	l2=aut_taps;
	i3=dm(i3_ptr);
	l3=delay_taps;

{-------------- Decay one coefficient only ----------------------------------}
{  NOTE: Upon reset, fractional arithmetic mode is enabled 		     }

	m5=1; m6=0;
	my0=dm(decay_aut); 		{ load decay value }
	  			        { to work must be circular!!!!! }
	i4=dm(r_ar2); l4=aut_taps;	{ load coefficient }
	mx0=pm(I4,m6);			{ (m6=0) }
	mr=mx0*my0(SS);    		{ decay * coeff }
	pm(i4,m5)=mr1;		  	{ (m5=1) save decayed coeff }
	dm(r_ar2)=i4;			{ save updated coeff pointer }

{------------- Tweak the coefficients ---------------------------------------}
	m1=1;
	i4=^coeff; l4=aut_taps;
	my0=dm(e_beta1); mx0=dm(i2,m1);
	cntr=aut_taps-1;
	do tweak1 until ce;
		mr=mx0*my0(SS);		{ delay-line data * e_beta }
		ay1=pm(i4,m6); 		{ (m6=0) next coeff. }
		mx0=dm(i2,m1);		{ next data }
		ar=mr1+ay1;
tweak1:		pm(i4,m5)=ar; 		{ (m5=1) save coeff. }

{-------------- Execute the FIR filter code ---------------------------------}
 
	i4=^coeff; l4=aut_taps;		{ set up adresses }  

	cntr=aut_taps-1;
	mr=0, mx0=dm(i2,m1), my0=pm(i4,m5);
	do conv1 until ce;
conv1:		mr=mr+mx0*my0(SS), mx0=dm(i2,m1), my0=pm(i4,m5);
	mr=mr+mx0*my0(RND);
	if MV sat mr;
	dm(output) = mr1;              	{ save FIR output result }
					{ Assume denoise function }
{--------------	Compute Error -----------------------------------------------}

	ax0=mr1;
	ay0=dm(sample);
	ar=ay0-ax0;			{ Error	= D - Y }
        dm(Error) = ar;              	{ Save Error for notching }
	my0=dm(beta_aut);
	mr=ar*my0(SS);			{ e_beta = Error * Beta }
	dm(e_beta1)=mr1;		{ save new e_beta }

{------------- Wrap up and return -------------------------------------------}
{     Autonotch requires a 63 tap delayed     signal to be put into X array  }
{     Denoise uses only  a single tap delayed signal to be put into X array  }

	m2=0;
	ar=dm(sample);	     		{ get raw input sample }
	dm(i3,m1)=ar;	   		{ (M1=1) update 63 tap input delay   }

	ar=dm(i3,m2);	   		{ (M2=0) Auto notch                  }
	dm(i2,m1)=ar;			{ (M1=1)                             }	

	dm(i2_ptr)=i2;			{ save registers }
	dm(i3_ptr)=i3;

	i2=dm(save_i2);	  		{ restore i2 and i3 }
	i3=dm(save_i3);
	
	ax0=dm(Error);			{ output signal }	  
	DM(sample)=ax0;	

	dm(notched_sample)=ax0;		{ notched output }

{----------------------------------------------------------------------------}
{                       Denoiser processing				     }
{        Input sample is in ax0, result goes back into ax0	    	     }
{----------------------------------------------------------------------------}
{*** now we must determine if the denoiser routine is fed from the notch or  }
{*** or from raw data - in any event, it will be run to reduce any "pop"     }
{*** if/when later activated.                                                }
{*** ax0 contains the notch result upon entry to this section of code        }

	ar=dm(SSB_switch);		{ if mode is not SSB, use raw }
	none=PASS ar;
	if EQ jump no_notch;

	ar=dm(Autonotch_switch);	{  it is SSB, is notch on? }
	none=PASS ar;
	if NE jump denoise_it;		{ yes, use notched sample }

no_notch:
	ax0=dm(unprocessed_sample);	{ no, setup as if we hadn't notched }
	dm(sample)=ax0;

{ ------------- Denoise - in or out ? ---------------------------------------}
{*** always run samples through here to eliminate "pop" when later activated }

denoise_it:
	i1=dm(i1_ptr);			{ set up registers }
	l1=dm(denoise_taps);

{-------------- Decay one coefficient only ----------------------------------}
{  NOTE: Upon reset, fractional arithmetic mode is enabled 		     }

	m5=1; m6=0;
	my0=dm(decay); 			{ load decay value                   }
	  			        { to work must be circular!!!!!      }
	i4=dm(r_ar1); l4=dm(denoise_taps);	{ load coefficient, length   }
	mx0=pm(i4,m6);			{ (M6=0)                             }
	mr=mx0*my0(SS);    		{ decay * coeff                      }
	pm(i4,m5)=mr1;		  	{ (M5=1) save decayed coeff.         }
	dm(r_ar1)=I4;			{ save updated coeff. pointer        }

{------------- Tweak the coefficients ---------------------------------------}
	m1=1;
	i4=^coeff1; l4=dm(denoise_taps);
	my0=dm(e_beta); mx0=dm(i1,m1);
	cntr=dm(denoise_tapsm1);
	do tweak until ce;
		mr=mx0*my0(SS);		{ delay-line data * e_beta           }
		ay1=pm(i4,m6); 		{ (M6=0) next coeff.                 }
		mx0=dm(i1,m1);		{ next data                          }
		ar=mr1+ay1;
tweak:		pm(i4,m5)=ar; 		{ (M5=1) save coeff.                 }

{-------------- Execute the FIR filter code ---------------------------------}
	i4=^coeff1; l4=dm(denoise_taps); { set up adresses                   }  

	cntr=dm(denoise_tapsm1);
	mr=0, mx0=dm(i1,m1), my0=pm(i4,m5);
	do conv until ce;
conv:		mr=mr+mx0*my0(SS), mx0=dm(i1,m1), my0=pm(i4,m5);
	mr=mr+mx0*my0(RND);
	if MV sat mr;
					{ Assume denoise function }
	dm(output) = mr1;              	{ save FIR output result }

{--------------	Compute Error ---------------------------------------------}

	ax0=mr1;
	ay0=dm(sample);
	ar=ay0-ax0;			{ Error	= D - Y }
	my0=dm(beta);
	mr=ar*my0(SS);			{ e_beta = Error * Beta }
	dm(e_beta)=mr1;			{ save new e_beta }

{------------- Wrap up routine ----------------------------------------------}
{     Denoise uses only  a single tap delayed signal to be put into X array  }

	ar=dm(sample);	     
	dm(i1,m1)=ar;			{ (M1=1)                             }	

	ar=dm(output);			{ Daisy chain filters }

{------------- adjust SSB denoise gain ------------------------------------------}

	sr0=dm(CW_switch);		{ if mode is CW, adjust with its gains }
	none=PASS sr0;
	if NE jump denoise_adjust_CW;

	se=dm(denoiser_large_gain);	{ do shift for large steps }
	si=ar;
	sr=ashift si (HI);
	my0=dm(denoiser_small_gain);	{ do multiply for fine steps }
	mr=sr1 * my0 (SS);
	if MV sat mr;			{ limit amplitude }
	ar=mr1;				{ fetch result of attenuator }
	jump denoise_adjusted;		{ done with SSB mode gain adjustment }


{------------- adjust CW denoise gain ----------------------------------------}

denoise_adjust_CW:
	se=dm(autonotch_large_gain);	{ do shift for large steps }
	si=ar;
	sr=ashift si (HI);
	my0=dm(autonotch_small_gain);	{ do multiply for fine steps }
	mr=sr1 * my0 (SS);
	if MV sat mr;			{ limit amplitude }
	ar=mr1;				{ fetch result of attenuator }

denoise_adjusted:
	dm(denoised_sample)=ar;		{ denoised }

	dm(i1_ptr)=i1;			{ save registers }

{-------------- Decide which value to return --------------------------------}
{ denoise  autonotch  return                                                 }
{   on       on       denoised_sample                                        }
{   on       off      denoised_sample                                        }
{   off      on       notched_sample                                         }
{   off      off      unprocessed sample                                     }
{----------------------------------------------------------------------------}

	ax0=dm(unprocessed_sample);	{ default case }

	ar=dm(DATA_switch);		{ if mode is data, we're done }
	none=PASS ar;
	if NE rts;

{ mode is not data, so notch is allowed }

	ar=dm(CW_switch);		{ if mode is CW, can't notch }
	none=PASS ar;
	if NE jump check_if_denoise;

check_if_notch: 			{ mode is SSB }

	ar=dm(Autonotch_switch);	{ use the notched sample? }
	none=PASS ar;
	if EQ jump check_if_denoise;	{ no, check denoiser switch }

	ax0=dm(notched_sample);		{ notch true, update return value }

check_if_denoise:
	ar=dm(Denoise_switch);		{ use the denoised sample? }
	none=PASS ar;
	if EQ rts;			{ no - already have correct value }

	ax0=dm(denoised_sample);	{ yes - update return value }

	rts;

{----------------------------------------------------------------------------}
.ENDMOD;

