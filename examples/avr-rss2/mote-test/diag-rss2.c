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
 *
 * Author  : Robert Olsson robert@radio-sensors.com
 * Created : 2015-11-22
 */

/**
 * \file
 *         HW test application for RSS2 mote
 */
#define VERSION       "1.3-2018-10-04"

#include <stdio.h>
#include <avr/eeprom.h>
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "sys/etimer.h"
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/battery-sensor.h"
#include "dev/temp-sensor.h"
#include "dev/temp_mcu-sensor.h"
#include "dev/light-sensor.h"
#include "dev/pulse-sensor.h"
#include "dev/bme280/bme280-sensor.h"
#include "dev/button-sensor.h"

#define T_OW_TEMP0       ((uint32_t) ((uint32_t) 1)<<0)
#define T_OW_TEMP1       ((uint32_t) ((uint32_t) 1)<<1)
#define T_P0             ((uint32_t) ((uint32_t) 1)<<2)
#define T_P1             ((uint32_t) ((uint32_t) 1)<<3)
#define T_PP             ((uint32_t) ((uint32_t) 1)<<4)
#define T_V_IN           ((uint32_t) ((uint32_t) 1)<<5)
#define T_V_MCU          ((uint32_t) ((uint32_t) 1)<<6)
#define T_T_MCU          ((uint32_t) ((uint32_t) 1)<<7)
#define T_A1             ((uint32_t) ((uint32_t) 1)<<8)
#define T_A2             ((uint32_t) ((uint32_t) 1)<<9)
#define T_LIGHT          ((uint32_t) ((uint32_t) 1)<<10)
#define T_EUI64          ((uint32_t) ((uint32_t) 1)<<11)
#define T_BME280         ((uint32_t) ((uint32_t) 1)<<12)
#define T_RTC            ((uint32_t) ((uint32_t) 1)<<13)
#define T_RADIO          ((uint32_t) ((uint32_t) 1)<<14)
#define T_DC_IN          ((uint32_t) ((uint32_t) 1)<<15)
#define T_BUTTON         ((uint32_t) ((uint32_t) 1)<<16)

#define DEF_TEST 0 
#define END_OF_FILE 26

extern void handle_serial_input(const char *line);

volatile uint32_t test = DEF_TEST;  /* default test mask */
uint32_t EEMEM ee_test = DEF_TEST;

#if 1
uint32_t test_mask =   (T_BUTTON|T_RTC|T_BME280|T_EUI64|T_P0|T_P1|T_V_MCU|T_T_MCU|T_V_IN|T_A1|T_A2|T_LIGHT|T_RADIO);
#else
uint32_t test_mask =  (T_OW_TEMP0|T_OW_TEMP1|T_P0|T_P1|T_V_IN|T_A1|T_A2|T_LIGHT|T_EUI64|T_RTC);
#endif

struct {
  uint32_t cs;
  double v_in;
  double a1;
  double a2;
  double light;
  int p0;
  int p1;
  int button;
} init;

struct {
  uint32_t rx_pkts;
  uint32_t tx_pkts;
} stats;

int debug = 0;

PROCESS(example_abc_process, "ABC example");
PROCESS(rss2_test_process, "rss2 test process");
PROCESS(programmable_power, "programmable power process");
PROCESS(serial_in, "cli input process");
AUTOSTART_PROCESSES(&rss2_test_process, &serial_in, &example_abc_process);

static struct etimer et, pp;
extern uint16_t ledtimer_red, ledtimer_yellow;
int ok;
uint32_t cs1, cs2;
int cs_init, retest;

void print_version(void)        
{
  printf("%s", VERSION);
}

void static
print_passed(void)
{
  printf("PASSED! BOTH LEDS SHOULD BE FLASHING. RESULT STORED IN EEPROM\n");
}

static void
blink(void)
{
  ledtimer_red = ledtimer_yellow = 1000;
  leds_on(LEDS_YELLOW);

  if(test == test_mask) {
    print_passed();
    eeprom_write_block((void*) &test,     (void*)&ee_test, sizeof(uint32_t));
  }
}

static void
test_i2c(void)
{

  if(! (i2c_probed & I2C_AT24MAC)) {
    printf("ERROR I2C EUI64 ADDR NOT FOUND\n");
  }

  if((test_mask & T_BME280) &&  ! (i2c_probed & I2C_BME280)) {
    printf("ERROR I2C BME280 SENSOR NOT FOUND\n");
  }
}

int
read_bme280(void)
{    
  uint16_t t = bme280_sensor.value(BME280_SENSOR_TEMP);
  uint16_t rh = bme280_sensor.value(BME280_SENSOR_HUMIDITY);
  uint16_t p =bme280_sensor.value(BME280_SENSOR_PRESSURE);
  
  int res = 0;

  if( t > 5 && t < 37 ) {
    if( rh > 5 && rh < 100 ) {
      if( p> 9000 && rh < 12000 ) {
	res = 1;
      }
    }
  }
  printf(" BME280_TEMP=%-d", t);
  printf(" BME280_RH=%-d", rh);
  printf(" BME280_P=%-d\n", p);

  return res;
}

static void
test_bme280(void)
{
  if(read_bme280())
    test |= T_BME280;
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
test_v_mcu(void)
{
  double v;
  v = ((double) battery_sensor.value(0))/1000.;
  if( v > 2.85 && v < 3.12 ) {
    test |= T_V_MCU;
    blink();
    if(debug)
      printf("OK V_MCU\n");
  }
}

static void
test_t_mcu(void)
{
  double t;
  t = ((double) temp_mcu_sensor.value(0)/10.);
  if( t > 15 && t < 38 ) {
    test |= T_T_MCU;
    blink();
    if(debug)
      printf("OK T_MCU\n");
  }
}

static void
test_button(void)
{
  if( init.button != button_sensor.value(0) ) {
    test |= T_BUTTON;
    blink();
    if(debug)
      printf("OK BUTTON (S2)\n");
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

void at24mac_print(int eof)
{
  uint8_t  i, buf[16];
  
  memset(buf, 0, sizeof(buf));
  i2c_read_mem(I2C_AT24MAC_ADDR, 0x98, buf, 8);
  for(i=0; i < 7; i++)
    printf("%02x", buf[i]);
  printf("%02x\n", buf[7]);

  if(eof) 
    printf("%c", END_OF_FILE); /* For tty_talk ID printing */
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
print_test_values(uint32_t t)
{
  if(t != test_mask) 
    printf("NOT TESTED:");

  if(test_mask & T_P0 && (t & T_P0) == 0)
    printf(" P0");

  if(test_mask & T_P1 && (t & T_P1) == 0)
    printf(" P1");

  if(test_mask & T_V_IN && (t & T_V_IN) == 0)
    printf(" V_IN");

  if(test_mask & T_V_MCU && (t & T_V_MCU) == 0)
    printf(" V_MCU");

  if(test_mask & T_T_MCU && (t & T_T_MCU) == 0)
    printf(" T_MCU");

  if(test_mask & T_EUI64 && (t & T_EUI64) == 0)
    printf(" EU64");

  if(test_mask & T_LIGHT && (t & T_LIGHT) == 0)
    printf(" LIGHT");

  if(test_mask & T_OW_TEMP0 && ((t & T_OW_TEMP0) == 0))
    printf(" OW_TEMP0");

  if(test_mask & T_OW_TEMP1 && ((t & T_OW_TEMP1) == 0))
  printf(" OW_TEMP1");

  if(test_mask & T_A1 && (t & T_A1) == 0)
    printf(" A1");

  if(test_mask & T_A2 && (t & T_A2) == 0)
    printf(" A2");

  if(test_mask & T_EUI64 && (t & T_EUI64) == 0)
    printf(" I2C-EUI64"); // KOLLA

  if(test_mask & T_BME280 && (t & T_BME280) == 0)
    printf(" I2C-BME280");

  if(test_mask & T_BUTTON && (t & T_BUTTON) == 0)
    printf(" BUTTON");

  if(test_mask & T_RADIO && (t & T_RADIO) == 0)
    printf(" RADIO");

  if(t != test_mask) 
    printf("\n");

  return  0;
}


void
test_status(void)
{
  if(test == test_mask) {
    print_passed();
    print_test_values(test);
  }
}

static int
test_values(void)
{
  uint32_t t;

  t = test & test_mask;

  if(debug)
    printf("t=0x%lx test_values=0x%lx test_mask=0x%lx\n", t, test, test_mask);
  
  print_test_values(t);

  if((t & T_P0) == 0)
    test_p0();

  if((t & T_P1) == 0)
    test_p1();

  if((t & T_V_IN) == 0)
    test_v_in();

  if((t & T_V_MCU) == 0)
    test_v_mcu();

  if((t & T_T_MCU) == 0)
    test_t_mcu();

  if((t & T_EUI64) == 0)
    test_eui64();

  if((t & T_LIGHT) == 0)
    test_light();

  if(((test_mask & T_OW_TEMP0) && (t & T_OW_TEMP0) == 0))
    test_ow_temp0();

  if(((test_mask & T_OW_TEMP1) && (t & T_OW_TEMP1) == 0))
    test_ow_temp1();

  if((t & T_A1) == 0)
    test_a1();

  if((t & T_A2) == 0)
    test_a2();

  if((t & T_BUTTON) == 0)
    test_button();

  test_i2c();

  return  0;
}

static void
init_test(void)
{
  init.cs = clock_seconds();

  init.v_in =  adc_read_v_in();

  init.a1 = adc_read_a1();
  if(init.a1 != 0) 
    printf("ERROR A1 NOT 0\n");

  init.a2 = adc_read_a2();
  if(init.a2 != 0) 
    printf("ERROR A2 NOT 0\n");

  init.p0 = pulse_sensor.value(0);
  if(init.p0 != 0) 
    printf("ERROR P0 NOT 0\n");

  init.p1 = pulse_sensor.value(1);
  if(init.p1 != 0) 
    printf("ERROR P1 NOT 0\n");

  init.light = light_sensor.value(0);

  init.button = button_sensor.value(0);
}

void
read_values(void)
{
  char serial[16];
  uint32_t cs;
  int i;

  cs = clock_seconds();
  if(debug) 
    printf(" Clock Seconds=%-lu\n", cs);
 
 /* Read out rss2 unique 128 bit ID */
  at24mac_print(0);
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
  printf(" BUTTON=%-d", button_sensor.value(0));
  printf("\n");
}

uint32_t utmp32;


void
force_retest(void)
{
  printf("TEST\n");

      retest = 1;
      test = 0;
      eeprom_write_block((void*)&test, (void*)&ee_test, sizeof(test));
}
PROCESS_THREAD(rss2_test_process, ev, data)
{
  PROCESS_BEGIN();

  eeprom_read_block((void*) &utmp32,    (const void*)&ee_test, sizeof(test));
  if( utmp32 == 0xFFFFFFFF) {
    utmp32 = 0;
    eeprom_write_block((void*)&utmp32, (void*)&ee_test, sizeof(test));
  }
  else {
    test = utmp32;
    if(debug) 
      printf("LAST RESULT=0x%lx\n",  utmp32);

    if(test == test_mask) { 
      printf("PASSED LAST TEST\n");
      retest = 0;
    }
    else {
      retest = 1;
    }
  }

  print_version();
  SENSORS_ACTIVATE(temp_sensor);
  SENSORS_ACTIVATE(battery_sensor);
  SENSORS_ACTIVATE(temp_mcu_sensor);
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(pulse_sensor);
  SENSORS_ACTIVATE(button_sensor);

  if( i2c_probed & I2C_BME280 ) {
    SENSORS_ACTIVATE(bme280_sensor);
    read_bme280();
  }

  etimer_set(&et, CLOCK_SECOND/4);
  while(!retest) {
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    etimer_set(&et, CLOCK_SECOND/2);
    
    if(test == test_mask) {
      ledtimer_red = ledtimer_yellow = 1000;
      leds_on(LEDS_RED);
      leds_on(LEDS_YELLOW);
    }
  }

  if(1) {
    /* Reset for Retest */
    test = 0;
    eeprom_write_block((void*)&test, (void*)&ee_test, sizeof(test));
  }

  if( i2c_probed & I2C_BME280 ) {
    test_bme280();
  }

  leds_init(); 
  init_test();
  test_i2c();
  process_start(&programmable_power, NULL);
  read_values();
  test_values();
  
#define DELAY 5

  /* 
   * Delay in sec 
   * Gives a chance to trigger some pulses
   */
  
  printf("Trigger %d sec RTC test\n", DELAY);
  cs1 = clock_seconds();
  etimer_set(&et, CLOCK_SECOND * DELAY);
  cs_init = 0;
  
  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&et));

    test_values();
    cs2 = clock_seconds();
    
    if(cs_init == 0 && (cs2-cs1 == DELAY)) {
      cs_init = 1;
      test |= T_RTC;
      printf("Trigger %d sec RTC done\n", DELAY);
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

PROCESS_THREAD(programmable_power, ev, data)
{
  PROCESS_BEGIN();

  DDRE |= (1<<PWR_1);

  etimer_reset(&pp);
  etimer_set(&pp, CLOCK_SECOND/4);

  while(1) {

    PROCESS_WAIT_UNTIL(etimer_expired(&pp));
    if(debug > 2) 
      printf("In programmable power\n");
    
    PORTE ^= (1<<PWR_1);

    etimer_reset(&pp);
    etimer_set(&pp, CLOCK_SECOND/2);
  }
  PROCESS_END();
}
PROCESS_THREAD(serial_in, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    handle_serial_input((const char *) data);
  }
  PROCESS_END();
}

static void
abc_recv(struct abc_conn *c)
{
  test |= T_RADIO;
  stats.rx_pkts++;
  if(debug) 
    printf("abc message received '%s'\n", (char *)packetbuf_dataptr());
}

static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;

PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(abc_close(&abc);)
  PROCESS_BEGIN();
  abc_open(&abc, 128, &abc_call);

  while(1) {

    etimer_set(&et, CLOCK_SECOND * 4);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    packetbuf_copyfrom("Hello", 6);
    abc_send(&abc);
    stats.tx_pkts++;
  }
  PROCESS_END();
}
