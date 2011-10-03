KDSP2 DSP Source Code

In order to use this code, you must re-assemble it, link it, split it
and merge the resulting hex image into the large hex image included in this
archive.  The first 8K bytes of the image created with this source will
replace the first 8K bytes of the KDSP2INIT.HEX file.  You must use a text editor
to accomplish this replacement.

Version 3.17 of the DSP code is functionally identical with 3.16 code distributed
with the KDSP2 product.  Version 3.18 fixes a bug in the LMS routines Beta and Decay
coefficient handling during re-initialization.  The 3.18 image was created with this
source file set.

For an inexpensive DSP development system suitable for developing with this code,
please see www.kk7p.com or www.tapr.org

The files included here are:

MAKEASP.BAT	MS-DOS batch file for making a new KDSP2 code image

ASP.DSP		Assembler source code for the main Audio Signal Processor module
ISR.DSP		Assembler source code for the Interrupt Service Routine module
LMS.DSP		Assembler source code for the LMS (denoise and notch) module
UART.DSP	Assembler source code for the serial port module

UART.INC	Assembler source file with definitions for UART module
DSP.INC		Assembler source file with definitions for DSP

EZKIT_LT.ACH	Architecture file required for linking

CW8060.DAT	Coefficient file for default CW filter
DA2202.DAT	Coefficient file for default DATA filter
DEC48.DAT	Coefficient file for decimator/interpolator
SB2502.DAT	Coefficient file for default SSB filter

KDSP2INIT.HEX	Intel HEX format image of DSP Flash versin 3.17

GNU_GPL.TXT	GNU General Public License which covers the soruce code released in this archive.
README.TXT	This file

This code falls under the GNU General Public License included in this archive.

June 06, 2003

[end]
