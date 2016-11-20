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
 *         A simple application testing the RSS2 mote
 */
#define VERSION       "1.0-2016-09-05"

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

static int
test_values(void)
{
  int i, ret;
  uint16_t ii;
  double  t, v;

  unsigned char eui64[8];

  printf("TESTING\n");

  i2c_at24mac_read((char *)&eui64, 1);

  /* Address Chip Atmels OUI */
  if( eui64[0] == 0xfc && eui64[1] == 0xc2 && eui64[2] == 0x3d) {
    ret = 1;
    printf("OK address chip\n");
  }
  else {
    printf("ERR address chip\n");
    for(i=0; i < 6; i++)
      printf("%02x", eui64[i]);
    printf("%02x\n", eui64[7]);
    return 0;
  }

  v = (double) adc_read_v_in();
  if( v > 4.7 && v < 5.3 ) {
    ret = 1;
    printf("OK V_IN\n");
  }
  else {
    printf("ERR V_IN\n");
    return  0;
  }

  ii = light_sensor.value(0);
  if( ii > 5 && ii < 350 ) {
    ret = 1;
    printf("OK light sensor\n");
  }
  else {
    printf("ERR light sensor\n");
    return  0;
  }
  return ret;

  /* Temp sensor */
  t = ((double) temp_sensor.value(0)/100.);
  if( t > 3 && t < 35 ) {
    ret = 1;
    printf("OK temp sensor\n");
  }
  else {
    printf("ERR temp sensor\n");
    return  0;
  }
}

static void
read_values(void)
{
  char serial[16];
  uint32_t cs;
  int i;

  cs = clock_seconds();
  printf(" Clock Seconds=%-lu\n", cs);
 
 /* Read out rss2 unique 128 bit ID */
  i2c_at24mac_read((char *) &serial, 0);
  printf("128_bit_ID=");
  for(i=0; i < 15; i++)
    printf("%02x", serial[i]);
  printf("%02x\n", serial[15]);
  printf("T=%-5.2f", ((double) temp_sensor.value(0)/100.));
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

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rss2_test_process, ev, data)
{
  PROCESS_BEGIN();

  print_version();
  SENSORS_ACTIVATE(temp_sensor);
  SENSORS_ACTIVATE(battery_sensor);
  SENSORS_ACTIVATE(temp_mcu_sensor);
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(pulse_sensor);
  leds_init(); 

  /* 
   * Delay 5 sec 
   * Gives a chance to trigger some pulses
   */

  /*
    TODO:FIX RTC test 
  */

    read_values();
    ok = test_values();
    printf("Trigger 5 sec RTC test\n");
    cs1 = clock_seconds();
    etimer_set(&et, CLOCK_SECOND * 5);
    cs_init = 0;
    while(1) {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      if(cs_init == 0) {
	cs2 = clock_seconds();
	printf("Trigger 5 sec RTC done\n");
	cs_init = 1;
	if(cs2-cs1 == 5) ;  
	else {
	  printf("ERR Clock %lu != 5\n", cs2-cs1);
	  ok = 0;
	}
      }

      etimer_reset(&et);
      etimer_set(&et, CLOCK_SECOND/2);
      ledtimer_red = ledtimer_yellow = 1000;
      if(ok) {
	leds_on(LEDS_RED);
	leds_on(LEDS_YELLOW);
      }
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
