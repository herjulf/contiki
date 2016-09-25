rss2-test
=========
Program to test various HW functions of the rss2-mote. Typically this is run 
after assembly. Serial bootloader is expcted first next rss2-test can be 
flashed via the serial bootloader. See separate avrdude command below.

LED result
----------
After 5 sec.

If test passed: Both yellow and red LED are blinking.

If not passed: No blinking LEDS. 
         Problem can be investigated via serial out.

Serial output
-------------
*******Booting Contiki-3.x-2729-gb2ef676*******
I2C: AT24MAC
EUI-64 MAC: fc-c2-3d-0-0-0-37-90
MAC=nullmac, RDC=nullrdc, NETWORK=sicslowpan, channel=26, check-rate-Hz=128, tx-power=0
RPL Enabled
Routing Enabled
1.0-2016-09-05 Clock Seconds=0
128_bit_ID=4415ff8518ff81ff90002224ff85ffa00020000100
T=21.31 V_MCU=3.0 V_IN=4.96 V_AD1=0.00 V_AD2=0.00 T_MCU=22.1 LIGHT=291 PULSE_0=0 PULSE_1=0
TESTING
OK address chip
OK temp sensor
OK V_IN
OK light sensor
Trigger 5 sec RTC test

Online
Addresses [4 max]                                    
fe80::fec2:3d00:0:3790
Neighbors [20 max]
  <none>
Routes [4 max]
  <none>
---------
Never-used stack > 21860 bytes

Trigger 5 sec RTC done            

Limitations
-----------
Many functions are not tested


Build
-----
make TARGET=avr-rss2


Flash
-----
avrdude -p m256rfr2 -c stk500v2 -P /dev/ttyUSB0 -b 38400 -e -U flash:w:rss2-test.avr-rss2


References
----------
https://github.com/herjulf/contiki/tree/mote-test
https://github.com/herjulf/contiki/tree/mote-test/examples/avr-rss2/mote-test

Full Program Setup
------------------
http://www.radio-sensors.com/download/rss2-test/rss2-program.jpg
