c:\adi_dsp\21xx\bin\asm21 -c asp.dsp -2181
c:\adi_dsp\21xx\bin\asm21 -c isr.dsp -2181
c:\adi_dsp\21xx\bin\asm21 -c uart.dsp -2181
c:\adi_dsp\21xx\bin\asm21 -c lms.dsp -2181
c:\adi_dsp\21xx\bin\ld21 asp.obj isr.obj uart.obj lms.obj -a ezkit_lt -e kdsp2 -x -g
c:\adi_dsp\21xx\bin\spl21 kdsp2 kdsp2 -loader -2181

