/*
 * Copyright (c) 2015, RAdio Sensors AB, Uppsala Sweden
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
 * This file is part of the Contiki operating system */

/**
 * \file
 *         Project specific configuration defines for example
 *
 * \author
 *         Robert Olsson <robert@radio-sensors.com>
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Netstack layers */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     tschmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nordc_driver
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154

/* IEEE802.15.4 frame version */
#undef FRAME802154_CONF_VERSION
#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154E_2012

/* TSCH and RPL callbacks */
#define RPL_CALLBACK_PARENT_SWITCH tsch_rpl_callback_parent_switch
#define RPL_CALLBACK_NEW_DIO_INTERVAL tsch_rpl_callback_new_dio_interval
#define TSCH_CALLBACK_JOINING_NETWORK tsch_rpl_callback_joining_network
#define TSCH_CALLBACK_LEAVING_NETWORK tsch_rpl_callback_leaving_network

#define TSCH_LOG_CONF_LEVEL 3

//#define TSCH_CONF_DEFAULT_TIMESLOT_LENGTH  15000

/* For test avoid 26 */
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 15, 25, 11, 20 }
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 25, 25 }
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 2

#if CONTIKI_TARGET_AVR_RSS2
#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 0
#define RF230_CONF_AUTOACK 0
#define RF230_CONF_AUTORETRIES 0
#define TSCH_CONF_ADAPTIVE_TIMESYNC 0
#define TSCH_CONF_HW_FRAME_FILTERING 0
#define TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS 0
#define TSCH_CONF_EB_PERIOD    (2 * CLOCK_SECOND)
//#define TSCH_CONF_RX_WAIT 4000
//#define TSCH_CONF_BASE_DRIFT_PPM 1997
//#define TSCH_CONF_TIMESYNC_REMOVE_JITTER 1
#endif

#endif /* PROJECT_CONF_H_ */
