diag-rss2
=========
A program to verfy  HW functions of the rss2-mote. 
Typically this is run after assembly. Serial bootloader 
is expcted first next diag-rss2 can be flashed via the 
serial bootloader. See separate avrdude command below.

Usage
------
Please read this intruction carefully. To test and 
verify an functional board should take less than one 
minute.

Serial commands
---------------
It is possible to give some commands via serial line.
NOTE! The commands ends with CONTROL-J (not CR) and 
typing is not echoed. Speed 38400 8bits now flowcontrol
just as with serial boot loader.

retest is one important command.

LED results
-----------
If test passed both yellow and red LED's are blinking.
A a message is printed on serial port:

PASSED! BOTH LEDS SHOULD BE FLASHING. RESULT STORED IN EEPROM

Testing is successfully terminated and led is flashing
restarting the test is possible via the restart command.

Problems can can be investigated via serial output.
         

Test and test setup
--------------------
To test some functionality of thw AD ports and interrupts 
pin the program needs some support. Interrupts need to be 
triggered and for AD ports (VCC) voltage must be applied.

Either manually or by attaching a special test-cable before 
running the test.

Some test details
-----------------
Both needs to be triggered by a touch to generate an interrupt
which is the critera for the program.

P0 (Pulse interupt) CON_INT5 (upper pin)
P1 (Pulse interupt comprator) CON_COMP (Rightmost pin)

A1 (AD1 ) CON_ADC1 (Rightmost pin)
A1 (AD1 ) ADC_2 (pin)

Special cable
-------------
This probably the fastest way to test the board.

Example: Test run
-----------------
*******Booting Contiki-3.x-3285-g1bfe900*******
I2C: AT24MAC BME280
EUI-64 MAC: fc-c2-3d-0-0-1-83-7e
PAN=0xABCD, MAC=nullmac, RDC=nullrdc, NETWORK=sicslowpan, channel=26, check-rate-Hz=128, tx-power=0
RPL Enabled
Routing Enabled
1.1-2017-05-02128_bit_ID=4415ff8518ff81ff90002124ffe7ffa00020000100
T0=24.00 T1=24.00 V_MCU=2.9 V_IN=4.99 V_AD1=0.00 V_AD2=0.00 T_MCU=26.6 LIGHT=284 PULSE_0=0 PULSE_1=0
NOT TESTED: P0 P1 V_IN EU64 LIGHT OW_TEMP0 OW_TEMP1 A1 A2 I2C-EUI64
Trigger 5 sec RTC test
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2   <<--- Applying PWR_PIN on P0
NOT TESTED: P0 A1 A2
NOT TESTED: P0 A1 A2  <<--- Applying PWR_PIN on P0
NOT TESTED: A1 A2
NOT TESTED: A1 A2  <<--- Applying PWR_PIN on A1
NOT TESTED: A2 
NOT TESTED: A2  <<--- Applying PWR_PIN on A2

PASSED! BOTH LEDS SHOULD BE FLASHING. RESULT STORED IN EEPROM

It's suggested to use PWR PIN (middle pin) from CON_ADC1 for
the the tests above.  Best to use a jumper cable. From:
* PWR PIN/CON_ADC -->  PWR_PIN on P0
* PWR PIN/CON_ADC -->  PWR_PIN on P1
* PWR PIN/CON_ADC -->  PWR_PIN on A1
* PWR PIN/CON_ADC -->  PWR_PIN on A2

A cable from PWR PIN/CON_ADC can connect to all P0/P1/A1/A2
at the same time

Build
-----
make TARGET=avr-rss2

Flash
-----
avrdude -p m256rfr2 -c stk500v2 -P /dev/ttyUSB0 -b 38400 -e -U flash:w:diag-rss2.avr-rss2

References
----------
https://github.com/herjulf/contiki/tree/mote-test
https://github.com/herjulf/contiki/tree/mote-test/examples/avr-rss2/mote-test

Full Program Setup
------------------
http://www.radio-sensors.com/download/diag-rss2/rss2-program.jpg
