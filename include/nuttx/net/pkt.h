/****************************************************************************
 * include/nuttx/net/pkt.h
 * Definitions for use with AF_PACKET sockets
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Daniel Laszlo Sitzer <dlsitzer@gmail.com>
 *
 * Includes some definitions that a compatible with the LGPL GNU C Library
 * header file of the same name.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_PKT_H
#define __INCLUDE_NUTTX_NET_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a uIP packet socket connection */

struct uip_pkt_conn
{
  dq_entry_t node;     /* Supports a double linked list */
  uint8_t    lmac[6];  /* The local Ethernet address in network byte order */
  uint8_t    ifindex;
  uint16_t   proto;
  uint8_t    crefs;    /* Reference counts on this instance */

  /* Defines the list of packet callbacks */

  struct uip_callback_s *list;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* uIP application functions
 *
 * Functions used by an application running of top of uIP. This includes
 * functions for opening and closing connections, sending and receiving
 * data, etc.
 *
 * Find a free connection structure and allocate it for use. This is
 * normally something done by the implementation of the socket() API
 */

struct uip_pkt_conn *uip_pktalloc(void);

/* Allocate a new packet socket data callback */

#define uip_pktcallbackalloc(conn)   uip_callbackalloc(&conn->list)
#define uip_pktcallbackfree(conn,cb) uip_callbackfree(cb, &conn->list)

/* Free a connection structure that is no longer in use. This should
 * be done by the implementation of close()
 */

void uip_pktfree(struct uip_pkt_conn *conn);

void uip_pktpoll(struct uip_driver_s *dev, struct uip_pkt_conn *conn);

int uip_pktinput(struct uip_driver_s *dev);

#endif /* __INCLUDE_NUTTX_NET_PKT_H */
