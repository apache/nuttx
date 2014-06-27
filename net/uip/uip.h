/****************************************************************************
 * net/uip/uip.h
 *
 *   Copyright (C) 2007-2009, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#ifndef _NET_UIP_UIP_H
#define _NET_UIP_UIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <arch/irq.h>

#include <nuttx/net/uip.h>
#include <nuttx/net/arp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uip_ipaddr_t g_alloneaddr;
extern const uip_ipaddr_t g_allzeroaddr;

/* Increasing number used for the IP ID field. */

extern uint16_t g_ipid;

/* Reassembly timer (units: deci-seconds) */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
extern uint8_t uip_reasstmr;
#endif

/* List of applications waiting for ICMP ECHO REPLY */

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING)
extern struct uip_callback_s *g_echocallback;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: uip_callbackinit
 *
 * Description:
 *   Configure the pre-allocated callback structures into a free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

void uip_callbackinit(void);

/****************************************************************************
 * Function: uip_callbackalloc
 *
 * Description:
 *   Allocate a callback container from the free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

FAR struct uip_callback_s *uip_callbackalloc(FAR struct uip_callback_s **list);

/****************************************************************************
 * Function: uip_callbackfree
 *
 * Description:
 *   Return a callback container to the free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

void uip_callbackfree(FAR struct uip_callback_s *cb,
                      FAR struct uip_callback_s **list);

/****************************************************************************
 * Function: uip_callbackexecute
 *
 * Description:
 *   Execute a list of callbacks.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

uint16_t uip_callbackexecute(FAR struct net_driver_s *dev, FAR void *pvconn,
                             uint16_t flags, FAR struct uip_callback_s *list);

/****************************************************************************
 * Send data on the current connection.
 *
 * This function is used to send out a single segment of TCP
 * data. Only applications that have been invoked by uIP for event
 * processing can send data.
 *
 * The amount of data that actually is sent out after a call to this
 * function is determined by the maximum amount of data TCP allows. uIP
 * will automatically crop the data so that only the appropriate
 * amount of data is sent. The function uip_mss() can be used to query
 * uIP for the amount of data that actually will be sent.
 *
 * Note: This function does not guarantee that the sent data will
 * arrive at the destination. If the data is lost in the network, the
 * application will be invoked with the UIP_REXMIT flag set.  The
 * application will then have to resend the data using this function.
 *
 * data A pointer to the data which is to be sent.
 *
 * len The maximum amount of data bytes to be sent.
 *
 ****************************************************************************/

void uip_send(FAR struct net_driver_s *dev, FAR const void *buf, int len);

#ifdef CONFIG_NET_IOB
struct iob_s;
void uip_iobsend(FAR struct net_driver_s *dev, FAR struct iob_s *buf,
                 unsigned int len, unsigned int offset);
#endif

#ifdef CONFIG_NET_PKT
void uip_pktsend(FAR struct net_driver_s *dev, FAR const void *buf,
                 unsigned int len);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* _NET_UIP_UIP_H */
