/*
 * Copyright (c) 2015, Copyright Robert Olsson / Radio Sensors AB  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 *
 * Author  : Robert Olsson robert@radio-sensors.com
 * Created : 2015-11-22
 */

/**
 * \file
 *         HW test application for RSS2 mote
 */
#define VERSION       "1.0-2017-05-01"

#include <avr/eeprom.h>
#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/battery-sensor.h"
#include "dev/temp-sensor.h"
#include "dev/temp_mcu-sensor.h"
#include "dev/light-sensor.h"
#include "dev/pulse-sensor.h"

#define T_OW_TEMP0       (1<<0)
#define T_OW_TEMP1       (1<<1)
#define T_P0             (1<<2)
#define T_P1             (1<<3)
#define T_PP             (1<<4)
#define T_V_IN           (1<<5)
#define T_A1             (1<<6)
#define T_A2             (1<<7)
#define T_LIGHT          (1<<8)
#define T_EUI64          (1<<9)
#define T_RTC            (1<<10)

#define DEF_TEST 0
volatile uint32_t test = DEF_TEST;  /* default test mask */
uint32_t EEMEM ee_test = DEF_TEST;

uint32_t test_mask =  (T_OW_TEMP0|T_OW_TEMP1|T_P0|T_P1|T_V_IN|T_A1|T_A2|T_LIGHT|T_EUI64|T_RTC);

struct {
  uint32_t cs;
  double v_in;
  double a1;
  double a2;
  double light;
  int p0;
  int p1;
} init;

int debug = 0;

void print_version(void)        
{
  printf("%s", VERSION);
}

/*---------------------------------------------------------------------------*/
PROCESS(rss2_test_process, "rss2 test process");
AUTOSTART_PROCESSES(&rss2_test_process);

static struct etimer et;
extern uint16_t ledtimer_red, ledtimer_yellow;
int ok;
uint32_t cs1, cs2;
int cs_init;


static void
blink(voiD)
{
  ledtimer_red = ledtimer_yellow = 1000;
  leds_on(LEDS_YELLOW);

  if(test == test_mask) {
     printf("PASSED! LEDS FLASHING. STORED IN EEPROM\n");
     eeprom_write_block((void*) &test,     (void*)&ee_test, sizeof(uint32_t));
  }
}

static void
test_v_in(void)
{
  double v;
  v = (double) adc_read_v_in();
  if( v > 4.7 && v < 5.3 ) {
    test |= T_V_IN;
    blink();
    if(debug)
      printf("OK V_IN\n");
  }
}

static void
test_a1(void)
{
  double v;
  v = adc_read_a1();
  if( init.a1 == 0 &&  v > 2.9 && v < 3.1 ) {
    test |= T_A1;
    blink();
    if(debug)
      printf("OK A1\n");
  }
}

static void
test_a2(void)
{
  double v;
  v = adc_read_a2();
  if( init.a2 == 0 &&  v > 2.9 && v < 3.1 ) {
    if(debug)
      printf("OK A2\n");
    test |= T_A2;
    blink();
  }
}

static void
test_p0(void)
{
  if( init.p0 == 0 &&  (pulse_sensor.value(0) > 0) ) {
    if(debug)
      printf("OK P0\n");
    test |= T_P0;
    blink();
  }
}

static void
test_p1(void)
{
  if( init.p1 == 0 &&  (pulse_sensor.value(1) > 0) ) {
    if(debug)
      printf("OK P1\n");
    test |= T_P1;
    blink();
  }
}

static void
test_eui64(void)
{
  unsigned char eui64[8];

  i2c_at24mac_read((char *)&eui64, 1);

  /* Address Chip Atmels OUI */
  if( eui64[0] == 0xfc && eui64[1] == 0xc2 && eui64[2] == 0x3d) {
    test |= T_EUI64;
    if(debug)
      printf("OK address chip\n");
    blink();
  }
}

static void
test_light(void)
{
  uint16_t ii;

  ii = light_sensor.value(0);

  if( ii > 5 && ii < 350 ) {
    test |= T_LIGHT;
    blink();
    if(debug) 
      printf("OK light sensor\n");
  }
}

static void
test_ow_temp0(void) 
{
  double t;
  /* Temp ow sensor */
  t = ((double) temp_sensor.value(0)/100.);
  if( t > 3 && t < 35 ) {
    test |= T_OW_TEMP0;
    blink();
    if(debug)
      printf("OK temp0 sensor\n");
  }
}

static void
test_ow_temp1(void) 
{
  double t;
  /* Temp ow sensor */
  t = ((double) temp_sensor.value(1)/100.);
  if( t > 3 && t < 35 ) {
    test |= T_OW_TEMP1;
    blink();
    if(debug)
      printf("OK temp1 sensor\n");
  }
}

static int
print_test_values(void)
{
  if(test != test_mask) 
    printf("TO TEST:");

  if((test & T_P0) == 0)
    printf(" P0");

  if((test & T_P1) == 0)
    printf(" P1");

  if((test & T_V_IN) == 0)
    printf(" V_IN");

  if((test & T_EUI64) == 0)
    printf(" EU64");

  if((test & T_LIGHT) == 0)
    printf(" LIGHT");

  if((test & T_OW_TEMP0) == 0)
    printf(" OW_TEMP0");

  if((test & T_OW_TEMP1) == 0)
    printf(" OW_TEMP1");

  if((test & T_A1) == 0)
    printf(" A1");

  if((test & T_A2) == 0)
    printf(" A2");

  if(test != test_mask) 
    printf("\n");

  return  0;
}

static int
test_values(void)
{
  if(debug)
    printf("test_values=0x%lx test_mask=9x%lx\n", test, test_mask);

  print_test_values();

  if((test & T_P0) == 0)
    test_p0();

  if((test & T_P1) == 0)
    test_p1();

  if((test & T_V_IN) == 0)
    test_v_in();

  if((test & T_EUI64) == 0)
    test_eui64();

  if((test & T_LIGHT) == 0)
    test_light();

  if((test & T_OW_TEMP0) == 0)
    test_ow_temp0();

  if((test & T_OW_TEMP1) == 0)
    test_ow_temp1();

  if((test & T_A1) == 0)
    test_a1();

  if((test & T_A2) == 0)
    test_a2();

  return  0;
}

static void
init_test(void)
{
  init.cs = clock_seconds();
  init.v_in =  adc_read_v_in();
  init.a1 = adc_read_a1();
  init.a2 = adc_read_a2();
  init.p0 = pulse_sensor.value(0);
  init.p1 = pulse_sensor.value(1);
  init.light = light_sensor.value(0);
}

static void
read_values(void)
{
  char serial[16];
  uint32_t cs;
  int i;

  cs = clock_seconds();
  if(debug) 
    printf(" Clock Seconds=%-lu\n", cs);
 
 /* Read out rss2 unique 128 bit ID */
  i2c_at24mac_read((char *) &serial, 0);
  printf("128_bit_ID=");
  for(i=0; i < 15; i++)
    printf("%02x", serial[i]);
  printf("%02x\n", serial[15]);
  printf("T0=%-5.2f", ((double) temp_sensor.value(0)/100.));
  printf(" T1=%-5.2f", ((double) temp_sensor.value(1)/100.));
  printf(" V_MCU=%-3.1f", ((double) battery_sensor.value(0))/1000.);
  printf(" V_IN=%-4.2f", adc_read_v_in());
  printf(" V_AD1=%-4.2f", adc_read_a1());
  printf(" V_AD2=%-4.2f", adc_read_a2());
  printf(" T_MCU=%-4.1f", ((double) temp_mcu_sensor.value(0)/10.));
  printf(" LIGHT=%-d", light_sensor.value(0));
  printf(" PULSE_0=%-d", pulse_sensor.value(0));
  printf(" PULSE_1=%-d", pulse_sensor.value(1));
  printf("\n");
}

uint32_t utmp32;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rss2_test_process, ev, data)
{
  PROCESS_BEGIN();


  eeprom_read_block((void*) &utmp32,    (const void*)&ee_test, sizeof(test));
  if( utmp32 == 0xFFFFFFFF) {
    utmp32 = 0;
    eeprom_write_block((void*)&utmp32, (void*)&ee_test, sizeof(test));
  }
  else {
    if(debug) 
      printf("LAST RESULT=0x%lx\n",  utmp32);

    if(utmp32 == test_mask) 
      printf("PASSED LAST TEST\n");
  }

  if(1) {
    /* Reset for Retest */
    utmp32 = 0;
    eeprom_write_block((void*)&utmp32, (void*)&ee_test, sizeof(test));
  }

  print_version();
  SENSORS_ACTIVATE(temp_sensor);
  SENSORS_ACTIVATE(battery_sensor);
  SENSORS_ACTIVATE(temp_mcu_sensor);
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(pulse_sensor);
  leds_init(); 
  init_test();
  read_values();
  test_values();
  
  /* 
   * Delay 5 sec 
   * Gives a chance to trigger some pulses
   */
  
  printf("Trigger 5 sec RTC test\n");
  cs1 = clock_seconds();
  etimer_set(&et, CLOCK_SECOND * 5);
  cs_init = 0;
  
  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    
    test_values();
    cs2 = clock_seconds();
    
    if(cs_init == 0 && (cs2-cs1 == 5)) {
      cs_init = 1;
      test |= T_RTC;
      printf("Trigger 5 sec RTC done\n");
    }

    etimer_reset(&et);
    etimer_set(&et, CLOCK_SECOND/2);
  
    if(test == test_mask) {
      ledtimer_red = ledtimer_yellow = 1000;
      leds_on(LEDS_RED);
      leds_on(LEDS_YELLOW);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
