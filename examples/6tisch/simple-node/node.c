/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 * \file
 *         A RPL+TSCH node able to act as either a simple node (6ln),
 *         DAG Root (6dr) or DAG Root with security (6dr-sec)
 *         Press use button at startup to configure.
 *
 * \author Jonas Sachwitz <j.sachwitz@gmx.de>
 * \author Simon Duquennoy <simonduq@sics.se>
*/

#include "contiki.h"
#include "sys/node-id.h"
#include "sys/log.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#include "net/mac/tsch/tsch.h"
#include "net/routing/routing.h"
#include "net/mac/tsch/tsch-slot-operation.h"
#include "simple-udp.h"
#include <stdio.h>
#include <string.h>

#include "tsch_measurement_template_EXCLUDES.h"

// Driver header file
#include <ti/drivers/GPIO.h>
 #include "Board.h"
//#include "dev/leds.h"

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

// UDP connection 
#define UDP_PORT 1234
static struct simple_udp_connection udp_conn;

// UDP connection Node A and Node B
#define UDP_PORT_NODES 2345
static struct simple_udp_connection udp_nodes_conn;

// Callback function for UDP message received
static void
udp_rx_callback(struct simple_udp_connection *c,
       const uip_ipaddr_t *sender_addr,
       uint16_t sender_port,
       const uip_ipaddr_t *receiver_addr,
       uint16_t receiver_port,
       const uint8_t *data,
       uint16_t datalen)
{
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_ON);
    struct tsch_asn_t asn = get_local_asn();
    printf("[INFO: TSCH-Measurement] {asn %02x.%08"PRIx32"} Received: %.*s\n", asn.ms1b, asn.ls4b, datalen, (char *) data);
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);
}

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "RPL Node");
AUTOSTART_PROCESSES(&node_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  int is_coordinator;
  int is_NodeA;
  PROCESS_BEGIN();
  is_coordinator = 0;
  is_NodeA = 1;

  // One-time init of GPIO driver
   GPIO_init();

#if CONTIKI_TARGET_COOJA || CONTIKI_TARGET_Z1
  is_coordinator = (node_id == 1);
#endif

  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }
  NETSTACK_MAC.on();

 /* Application Start*/

// Coordinator send UDP message every 10 ms with count to all nodes

  simple_udp_register(&udp_conn, UDP_PORT, NULL, UDP_PORT, udp_rx_callback);
  char str[80];  // Buffer for UDP messages

  static struct etimer et;
  etimer_set(&et, CLOCK_SECOND/10);

  if(is_coordinator) {
      while(1) {
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
          GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_ON);
          etimer_reset(&et);

          // Send UDP message to all nodes with count
          uip_ipaddr_t broadcast;
          uip_create_linklocal_allnodes_mcast(&broadcast);
          static uint32_t count = 0;

          GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);

          struct tsch_asn_t asn = get_local_asn();
          sprintf(str, "Message %lu\n\r", (unsigned long) count);
          simple_udp_sendto(&udp_conn, str, strlen(str), &broadcast);
          printf("[INFO: TSCH-Measurement] {asn %02x.%08"PRIx32"} | Broadcast Message %lu\n\r ", asn.ms1b, asn.ls4b, (unsigned long) count);
          count +=1;
      }

  } else if(is_NodeA) {

    // Node A send UDP to IP address of Node B every second
    simple_udp_register(&udp_nodes_conn, UDP_PORT_NODES, NULL, UDP_PORT_NODES, udp_rx_callback);

    while(1) {
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
          etimer_reset(&et);

          // destination IP address of Node B --> fe80::212:4b00:21a9:e601
          uip_ipaddr_t dest_ipaddr = {{0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4b, 0x00, 0x21, 0xa9, 0xe6, 0x01}};
          simple_udp_sendto(&udp_nodes_conn, "Hello Node B", strlen("Hello Node A"), &dest_ipaddr);
      }
  } else simple_udp_register(&udp_nodes_conn, UDP_PORT_NODES, NULL, UDP_PORT_NODES, udp_rx_callback);


  // One-time TI-DRIVERS Board initialization
  //Board_init();

  //SET_LED0();

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
