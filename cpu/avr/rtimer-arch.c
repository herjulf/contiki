/*
 * Copyright (c) 2017, KTH & Radio Sensors AB
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
 */

/**
 * \file
 *         AVR-specific rtimer code
 *         Using 32bit MAC Symbol Counter 
 * \author
 *         Robert Olsson
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "dev/leds.h"
#include <dev/watchdog.h>
#include <stdio.h>
#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "rtimer-arch.h"

extern uint32_t msc_get_counter();
extern uint32_t msc_sync();
volatile uint8_t rtimer_wait;

void
rtimer_arch_init(void)
{
  uint8_t sreg;
  sreg = SREG;
  cli ();

  SREG = sreg;
}

rtimer_clock_t
rtimer_arch_now(void) 
{
  return (rtimer_clock_t) msc_get_counter();
}

ISR (TIMER3_COMPA_vect) 
{
}

void
rtimer_arch_sleep(rtimer_clock_t howlong)
{
  cli();
  watchdog_stop();
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  msc_sync_counter(); /* Needed before sleep */

  rtimer_wait = 1;
  sei();
while(rtimer_wait)
    sleep_mode();
  watchdog_start();
}
