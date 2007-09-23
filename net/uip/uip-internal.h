/****************************************************************************
 * net/uip/uip-internal.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __UIP_INTERNAL_H
#define __UIP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <errno.h>
#include <arch/irq.h>
#include <net/uip/uip.h>

/****************************************************************************
 * Public Macro Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uip_ipaddr_t all_ones_addr;
extern const uip_ipaddr_t all_zeroes_addr;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Defined in uip_tcpconn.c *************************************************/

EXTERN void uip_tcpinit(void);
EXTERN struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf);
EXTERN struct uip_conn *uip_tcplistener(uint16 portno);
EXTERN struct uip_conn *uip_tcpaccept(struct uip_tcpip_hdr *buf);
EXTERN void uip_tcpnextsequence(void);

/* Defined in uip_udpconn.c *************************************************/

EXTERN void uip_udpinit(void);
EXTERN struct uip_udp_conn *uip_udpactive(struct uip_udpip_hdr *buf);

/* Defined in uip_listen.c **************************************************/

EXTERN void uip_listeninit(void);
EXTERN boolean uip_islistener(uint16 port);
EXTERN int uip_accept(struct uip_conn *conn, uint16 portno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __UIP_INTERNAL_H */
