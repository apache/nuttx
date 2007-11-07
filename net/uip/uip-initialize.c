/****************************************************************************
 * net/uip/uip-udppoll.c
 * Poll for the availability of UDP TX data
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <net/uip/uip.h>

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#if UIP_URGDATA > 0
void *uip_urgdata;           /* urgent data (out-of-band data), if present. */
uint16 uip_urglen;           /* Length of (received) urgent data */
#endif

/* The uip_flags variable is used for communication between the TCP/IP
 * stack and the application program.
 */

uint8 uip_flags;

/* uip_conn always points to the current connection. */

struct uip_conn *uip_conn;

#ifdef CONFIG_NET_UDP
struct uip_udp_conn *uip_udp_conn;
#endif   /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_STATISTICS
struct uip_stats uip_stat;
#endif

/* Increasing number used for the IP ID field. */

uint16 g_ipid;

const uip_ipaddr_t all_ones_addr =
#ifdef CONFIG_NET_IPv6
  {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
#else /* CONFIG_NET_IPv6 */
  {0xffffffff};
#endif /* CONFIG_NET_IPv6 */

const uip_ipaddr_t all_zeroes_addr =
#ifdef CONFIG_NET_IPv6
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
#else
  {0x00000000};
#endif

/* Reassembly timer (units: deci-seconds) */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
uint8 uip_reasstmr;
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_initialize
 *
 * Description:
 *   Perform initialization of the uIP layer
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void uip_initialize(void)
{
  /* Initialize the listening port structures */

  uip_listeninit();

  /* Initialize the TCP/IP connection structures */

  uip_tcpinit();

  /* Initialize the UDP connection structures */

#ifdef CONFIG_NET_UDP
  uip_udpinit();
#endif

  /* IPv4 initialization. */
}
#endif /* CONFIG_NET */

