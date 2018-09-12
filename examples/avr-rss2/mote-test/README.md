diag-rss2
=========
A program to verfy HW functions of the rss2-mote. 
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
Speed 38400 8bits now flowcontrol.

h for help
eui64 or ss for EUI64/MAC addresses
retest is one important command.

Requirements
-------------
* Jumper cable.  
* Addtional node with diag-rss2 for radio test.

LED results
-----------
If test passed both yellow and red LED's are blinking.
A a message is printed on serial port:

PASSED! BOTH LEDS SHOULD BE FLASHING. RESULT STORED IN EEPROM

Testing is successfully terminated and led is flashing
restarting the test is possible via the restart command.
Problems can be investigated via serial output.
         
Test procedure in two steps
---------------------------
1) boot from DC input/white connector using power adapter only
   if yellow LED should flash. 
2) board functional test diag-rss2 program

diag-rss2 tests
---------------
To test some functionality of thw AD ports and interrupts 
pin the program needs some manual interaction. 
* Interrupts need to be triggered. P0, P1 
* AD ports (VCC) voltage must be applied. A1, A2

Some test details
-----------------
Both needs to be triggered by a touch to generate an interrupt
which is the critera for the program.

It's suggested to use PWR PIN (middle pin) from CON_ADC1 for
the the tests above.  Best to use a jumper cable.
* PWR PIN/CON_ADC -->  PWR_PIN on P0
* PWR PIN/CON_ADC -->  PWR_PIN on P1
* PWR PIN/CON_ADC -->  PWR_PIN on A1
* PWR PIN/CON_ADC -->  PWR_PIN on A2

P0 (Pulse interupt) CON_INT5 (upper pin)
P1 (Pulse interupt comprator) CON_COMP (Rightmost pin)
A1 (AD1 ) CON_ADC1 (Rightmost pin)
A2 (AD2 ) ADC_2 (pin)

Example: Test run
=================
*******Booting Contiki-3.x-3304-gd922fce*******
Boot cause: External reset
I2C: AT24MAC BME280
EUI-64 MAC: fc-c2-3d-0-0-1-83-7e
PAN=0xABCD, MAC=nullmac, RDC=nullrdc, NETWORK=sicslowpan, channel=26, check-rate-Hz=128, tx-power=0
RPL Enabled
Routing Enabled
1.1-2017-05-02128_bit_ID=4415ff8518ff81ff90002124ffe7ffa00020000100
T0=24.00 T1=24.00 V_MCU=2.9 V_IN=4.99 V_AD1=0.00 V_AD2=0.00 T_MCU=26.6 LIGHT=284 PULSE_0=0 PULSE_1=0
NOT TESTED: P0 P1 V_IN EU64 LIGHT OW_TEMP0 OW_TEMP1 A1 A2 I2C-EUI64
Trigger 5 sec RTC test
NOT TESTED: P0 P1 A1 A2 BUTTON RADIO
NOT TESTED: P0 P1 A1 A2 BUTTON RADIO <--- Turning another node w. diag-rss2 pgm
NOT TESTED: P0 P1 A1 A2 BUTTON
NOT TESTED: P0 P1 A1 A2 BUTTON <--- Pressing S2 button
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2
NOT TESTED: P0 P1 A1 A2   <<--- Connecting PWR_PIN on P1
NOT TESTED: P0 A1 A2
NOT TESTED: P0 A1 A2  <<--- Connecting PWR_PIN on P0
NOT TESTED: A1 A2
NOT TESTED: A1 A2  <<--- Connecting PWR_PIN on A1
NOT TESTED: A2 
NOT TESTED: A2  <<--- Connecting PWR_PIN on A2

PASSED! BOTH LEDS SHOULD BE FLASHING. RESULT STORED IN EEPROM


Build (Not needed)
-----
make TARGET=avr-rss2

Flash
-----
avrdude -p m256rfr2 -c stk500v2 -P /dev/ttyUSB0 -b 38400 -e -U flash:w:diag-rss2.avr-rss2

References and code
-------------------
https://github.com/herjulf/contiki/tree/mote-test
https://github.com/herjulf/contiki/tree/mote-test/examples/avr-rss2/mote-test

Prebuilt application
--------------------
http://www.radio-sensors.com/download/diag-rss2/diag-rss2.avr-rss2


TODO 
----
DC-connector test. Verify boot from DC connector.

