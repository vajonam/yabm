Yet Another Blinds Minder

Arduino Code that opens and closes blinds similar to
http://homeawesomation.wordpress.com/2013/02/26/automated-window-blinds-with-arduino/
 
The difference is that input methods are different and the blinds are 
open during reasonable light conditions during the day but closes 
when direct sunlight hits the blinds and also closes at night 
time. This is inteded to be completely automatic and require no 
user intervention to open close the blinds. The blinds close 
gradually as light increase upto max threshold. Also there is code
devoted to smoothing the LDR inputs to produce a flutter/jitter
free opreation of the blinds.
 
