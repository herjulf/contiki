/*
 * Copyright (c) 2011-2016, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         Minimal and simple serial line API for ipv6 and RPL monitoring
 *         See help command to get an overview
 *
 * \Author
 *         Robert Olsson
 *
 * \Used code from border-router.c with authors
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *
 * \Used code from pdr.c with authors:
 *         Atis Elsts       <atis.elsts@bristol.ac.uk>
 *         Christian Rohner <christian.rohner@it.uu.se>
 *         Robert Olsson    <roolss@kth.se>
 */

#define CLI_VERSION "0.9-2017-03-13\n"

#if CONTIKI_TARGET_AVR_RSS2
#define radio_set_txpower rf230_set_txpower
#define radio_get_txpower rf230_get_txpower
#define radio_get_rssi    rf230_rssi
#endif

#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/mac/framer-802154.h"
#include "net/link-stats.h"
#include <string.h>
#include <stdlib.h>
#include "net/packetbuf.h"
#include "dev/serial-line.h"
#include "dev/i2c.h"
#include "sys/process.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if CONTIKI_TARGET_AVR_RSS2
#include <avr/wdt.h>
#endif

#ifdef CLI_CONF_COMMAND_PROMPT
#define CLI_COMMAND_PROMPT CLI_CONF_COMMAND_PROMPT
#else
#define CLI_COMMAND_PROMPT  "> "
#endif

#ifdef CLI_CONF_PROJECT
#define CLI_PROJECT CLI_CONF_PROJECT
#else
#define CLI_PROJECT  ""
#endif

extern int debug;
extern int retest;
extern void test_status(void);
extern void force_retest(void);

const char *delim = " \t\r,";
#define END_OF_FILE 26
uint8_t eof = END_OF_FILE;
uint8_t channel;

#define READY_PRINT_INTERVAL (CLOCK_SECOND * 5)

#if CLI_STANDALONE
PROCESS(cli, "cli app");
AUTOSTART_PROCESSES(&cli);
#endif

static void
print_help(void)
{
  printf("%s\n", CLI_PROJECT);
  printf("cli: version=%s", CLI_VERSION);
  printf("show version\n");
  printf("show version\n");
  printf("set debug  -- select debug info\n");
  printf("i2c       -- probe i2c bus\n");
  printf("help         -- this menu\n");
  printf("retest       -- restart test\n");
  printf("status       -- test status\n");
  printf("upgr         -- reboot via bootloader\n");

  printf("Uptime %lu sec\n", clock_seconds());
}
static uint8_t
radio_get_channel(void)
{
  radio_value_t chan;

  if(NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &chan) ==
     RADIO_RESULT_OK) {
    return chan;
  }
  printf("Err get_chan\n");
  return 0;
}
static void
radio_set_channel(uint8_t channel)
{
  if(NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, channel) ==
     RADIO_RESULT_OK) {
    printf("Err set_chan=%d\n", channel);
    return;
  }
}

static int
cmd_chan(uint8_t verbose)
{
  uint8_t tmp;
  char *p = strtok(NULL, delim);

  if(p) {
    tmp = atoi((char *)p);
    if(tmp >= 11 && tmp <= 26) {
      channel = tmp;
      radio_set_channel(channel);
    } else {
      printf("Invalid chan=%d\n", tmp);
      return 0;
    }
  }
  if(verbose) {
    printf("chan=%d\n", radio_get_channel());
  }
  return 1;
}
void
debug_cmd(char *p)
{
  debug = 1;
}
void
handle_serial_input(const char *line)
{
  char *p;
  /* printf("in: '%s'\n", line); */
  p = strtok((char *)&line[0], (const char *)delim);

  if(!p) {
    return;
  }

  printf("\n");

  /* Show commands */
  if(!strcmp(p, "sh") || !strcmp(p, "sho") || !strcmp(p, "show")) {
    p = strtok(NULL, (const char *)delim);
    if(p) {
      if(!strcmp(p, "ch") || !strcmp(p, "chan")) {
        cmd_chan(1);
      } else if(!strcmp(p, "v") || !strcmp(p, "ver")) {
        printf("%s", CLI_VERSION);
      }
    }
  }
  /* Set commands */
  else if(!strcmp(p, "sh") || !strcmp(p, "sho") || !strcmp(p, "show")) {
    p = strtok(NULL, (const char *)delim);
    if(p) {
      if(!strcmp(p, "de") || !strcmp(p, "debug")) {
        debug_cmd(p);
      }
    }
  }

#ifdef CONTIKI_TARGET_AVR_RSS2
  else if(!strcmp(p, "i2c")) {
    printf("I2C: ");
    i2c_probed = i2c_probe();
    printf("\n");
  } else if(!strcmp(p, "upgr") || !strcmp(p, "upgrade")) {
    printf("OK\n");
    printf("%c", eof);
    cli();
    wdt_enable(WDTO_15MS);
    while(1) ;
  }
#endif
  else if(!strcmp(p, "retest") || !strcmp(p, "r")) {
    force_retest();
  }
  else if(!strcmp(p, "status") || !strcmp(p, "s")) {
    test_status();
    printf("I2C: ");
    i2c_probed = i2c_probe();
    printf("\n");
    read_values();
    read_bme280();
  }
  else if(!strcmp(p, "help") || !strcmp(p, "h")) {
    print_help();
  } else { printf("Illegal command '%s'\n", p);
  }
  printf(CLI_COMMAND_PROMPT);
}
#ifdef CLI_STANDALONE

static struct etimer periodic;

PROCESS_THREAD(cli, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND);

  for(;;) {
    PROCESS_WAIT_EVENT();

    /* printf("event %u (%u) at %u, data %p\n", (uint16_t)ev, (uint16_t)serial_line_event_message, currentState, data); */
    printf("event %u (%u) at  data %p\n", (uint16_t)ev, (uint16_t)serial_line_event_message, data);

    if(etimer_expired(&periodic)) {
      etimer_set(&periodic, READY_PRINT_INTERVAL);
    }
    if(ev == PROCESS_EVENT_POLL) {
      etimer_set(&periodic, READY_PRINT_INTERVAL);
    }
  }
  PROCESS_END();
}

#endif
