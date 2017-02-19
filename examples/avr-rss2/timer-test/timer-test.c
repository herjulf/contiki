/**
 * \file
 *         Tests related to clocks and timers
 *         This is based on clock_test.c from the original sensinode port
 *
 * \author
 *         Zach Shelby <zach@sensinode.com> (Original)
 *         George Oikonomou - <oikonomou@users.sourceforge.net> (rtimer code)
 *
 */

/* Version for 16 bit timrs Robert Olsson */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include <dev/watchdog.h>
#include <stdio.h>
#include <avr/wdt.h>
/*---------------------------------------------------------------------------*/
#define TEST_CLOCK_DELAY_USEC 1
#define TEST_RTIMER           1
#define TEST_ETIMER           1
#define TEST_CLOCK_SECONDS    1
/*---------------------------------------------------------------------------*/
static struct etimer et;

#define RTIMER_RES 1000000/RTIMER_SECOND 

#if TEST_CLOCK_DELAY_USEC
static rtimer_clock_t start_count, end_count, diff;
#endif

#if TEST_CLOCK_SECONDS
static unsigned long sec;
#endif

#if TEST_ETIMER
static clock_time_t count;
#endif

#if TEST_RTIMER
static struct rtimer rt;
rtimer_clock_t rt_now, rt_for;
static clock_time_t ct;
#endif

static uint16_t i, j;
/*---------------------------------------------------------------------------*/
PROCESS(clock_test_process, "Clock test process");
AUTOSTART_PROCESSES(&clock_test_process);
/*---------------------------------------------------------------------------*/
#if TEST_RTIMER
void
rt_callback(struct rtimer *t, void *ptr)
{
  rt_now = RTIMER_NOW();
  ct = clock_time();
  printf("Task called at %lu (clock = %lu)\n", rt_now, ct);
}
#endif

#ifdef CONTIKI_TARGET_AVR_RSS2
static void
stop_avr_wdt(void)
{
    cli();
    watchdog_stop();
    wdt_disable();
    /* Clear WDRF in MCUSR*/
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    sei();
}
#endif

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(clock_test_process, ev, data)
{
  PROCESS_BEGIN();
#ifdef CONTIKI_TARGET_AVR_RSS2
  stop_avr_wdt();
#endif
  leds_init();

  etimer_set(&et,  CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  etimer_reset(&et);

  printf("RTIMER_SECOND = (%lu rtimer ticks):\n", RTIMER_SECOND);
  printf("sizeof rtimer_clock_t = %d\n", sizeof(rtimer_clock_t));
  printf("sizeof clock_time_t = %d\n", sizeof(clock_time_t));

 
#if TEST_CLOCK_DELAY_USEC
  //printf("clock_delay_usec test, (10,000 x i) usec:\n");
  i = 1;
  while(i < 7) {
    start_count = RTIMER_NOW();
    /* Investigate why this i needed */
    for(j=0; j < i; j++) {
      clock_delay_usec(10000);
    }
    end_count = RTIMER_NOW();
    diff = end_count - start_count;
    printf("Requested: %u usec, Real: %lu rtimer ticks = ~%lu us\n",
           10000 * i, diff, diff * RTIMER_RES);
    i++;
  }
#endif

#if TEST_RTIMER
  printf("Rtimer Test, 1 sec (%lu rtimer ticks):\n", RTIMER_SECOND);
  i = 0;
  while(i < 5) {
    etimer_set(&et, CLOCK_SECOND);
    printf("=======================\n");
    ct = clock_time();
    rt_now = RTIMER_NOW();
    rt_for = rt_now + RTIMER_SECOND;
    printf("Now=%lu (clock = %lu) - For=%lu\n", rt_now, ct, rt_for);
    if(rtimer_set(&rt, rt_for, 1, (rtimer_callback_t) rt_callback, NULL) !=
       RTIMER_OK) {
      printf("Error setting\n");
    }
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    i++;
  }
#endif

#if TEST_ETIMER
  printf("Clock tick and etimer test, 1 sec (%u clock ticks):\n",
         CLOCK_SECOND);
  i = 0;
  while(i < 10) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    count = clock_time();
    printf("%lu ticks\n", count);

    leds_toggle(LEDS_RED);
    i++;
  }
#endif

#if TEST_CLOCK_SECONDS
  printf("Clock seconds test (2s):\n");
  i = 0;
  while(i < 10) {
    etimer_set(&et, 2 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    sec = clock_seconds();
    printf("%lu seconds\n", sec);

    leds_toggle(LEDS_YELLOW);
    i++;
  }
#endif

  printf("Done!\n");  

  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    leds_toggle(LEDS_YELLOW);
    leds_toggle(LEDS_RED);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
