/*
 * Copyright (c) 2023, HS-Aalen
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
 */

/**
 * \author Jonas Sachwitz <j.sachwitz@gmx.de>
 */

#ifndef TSCH_MEASUREMENT_TEMPLATE_H_
#define TSCH_MEASUREMENT_TEMPLATE_H_

#include "contiki.h"
#include <stdio.h>
#include <inttypes.h>
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-asn.h"

// Driver header file
#include <ti/drivers/GPIO.h>
 #include "Board.h"
#include "dev/leds.h"

// define necessary, if Node A is Time Source from Node B
#define NODEB

// Debugging Macro for Slot Start with TI drvier
#define TSCH_DEBUG_SLOT_START() do { \
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON); \
} while(0)

// Debugging Macro for Slot End with TI driver
#define TSCH_DEBUG_SLOT_END() do { \
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF); \
} while(0)

// Function to get the actual ASN of the node
struct tsch_asn_t get_local_asn();

// print current asn of Node
void print_current_asn(void);



/* Old Code from the process*/

 /*
 // get local asn every second
  static struct etimer et;
  etimer_set(&et, CLOCK_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    print_current_asn();

    // Use Ti-Driver
    GPIO_toggle(Board_GPIO_LED0);
    //leds_toggle(LEDS_ALL) --> work only with hex, not with macro
  } */

#endif /* TSCH_MEASUREMENT_TEMPLATE_H_ */
