/****************************************************************************
 * net/uip/uip_internal.h
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

#ifndef _NET_UIP_UIP_INTERNAL_H
#define _NET_UIP_UIP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <arch/irq.h>
#include <nuttx/net/uip/uip.h>

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
extern "C" {
#else
#define EXTERN extern
#endif

/* Defined in uip_callback.c ************************************************/

void uip_callbackinit(void);
FAR struct uip_callback_s *uip_callbackalloc(struct uip_callback_s **list);
void uip_callbackfree(FAR struct uip_callback_s *cb, struct uip_callback_s **list);
uint16_t uip_callbackexecute(FAR struct uip_driver_s *dev, void *pvconn,
                             uint16_t flags, FAR struct uip_callback_s *list);

#ifdef CONFIG_NET_TCP
/* Defined in uip_tcpconn.c *************************************************/

void uip_tcpinit(void);
struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf);
struct uip_conn *uip_nexttcpconn(struct uip_conn *conn);
struct uip_conn *uip_tcplistener(uint16_t portno);
struct uip_conn *uip_tcpaccept(struct uip_tcpip_hdr *buf);

/* Defined in uip_tcpseqno.c ************************************************/

void uip_tcpsetsequence(FAR uint8_t *seqno, uint32_t value);
uint32_t uip_tcpgetsequence(FAR uint8_t *seqno);
uint32_t uip_tcpaddsequence(FAR uint8_t *seqno, uint16_t len);
void uip_tcpinitsequence(FAR uint8_t *seqno);
void uip_tcpnextsequence(void);

/* Defined in uip_tcppoll.c *************************************************/

void uip_tcppoll(struct uip_driver_s *dev, struct uip_conn *conn);

/* Defined in uip_udptimer.c ************************************************/

void uip_tcptimer(struct uip_driver_s *dev, struct uip_conn *conn, int hsec);

/* Defined in uip_listen.c **************************************************/

void uip_listeninit(void);
bool uip_islistener(uint16_t port);
int uip_accept(struct uip_driver_s *dev, struct uip_conn *conn, uint16_t portno);

/* Defined in uip_tcpsend.c *************************************************/

void uip_tcpsend(struct uip_driver_s *dev, struct uip_conn *conn,
                 uint16_t flags, uint16_t len);
void uip_tcpreset(struct uip_driver_s *dev);
void uip_tcpack(struct uip_driver_s *dev, struct uip_conn *conn,
                uint8_t ack);

/* Defined in uip_tcpappsend.c **********************************************/

void uip_tcpappsend(struct uip_driver_s *dev, struct uip_conn *conn,
                    uint16_t result);
void uip_tcprexmit(struct uip_driver_s *dev, struct uip_conn *conn,
                   uint16_t result);

/* Defined in uip_tcpinput.c ************************************************/

void uip_tcpinput(struct uip_driver_s *dev);

/* Defined in uip_tcpcallback.c *********************************************/

uint16_t uip_tcpcallback(FAR struct uip_driver_s *dev,
                         FAR struct uip_conn *conn, uint16_t flags);
#ifdef CONFIG_NET_TCP_READAHEAD
uint16_t uip_datahandler(FAR struct uip_conn *conn,
                         FAR uint8_t *buffer, uint16_t nbytes);
#endif

/* Defined in uip_tcpreadahead.c ********************************************/

#ifdef CONFIG_NET_TCP_READAHEAD
void uip_tcpreadahead_init(void);

struct uip_readahead_s;
FAR struct uip_readahead_s *uip_tcpreadahead_alloc(void);
void uip_tcpreadahead_release(FAR struct uip_readahead_s *readahead);
#endif /* CONFIG_NET_TCP_READAHEAD */

/* Defined in uip_tcpwrbuffer.c *********************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void uip_tcpwrbuffer_init(void);

struct uip_wrbuffer_s;
struct timespec;
FAR struct uip_wrbuffer_s *uip_tcpwrbuffer_alloc(FAR const struct timespec *abstime);
void uip_tcpwrbuffer_release(FAR struct uip_wrbuffer_s *wrbuffer);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
/* Defined in uip_udpconn.c *************************************************/

void uip_udpinit(void);
struct uip_udp_conn *uip_udpactive(struct uip_udpip_hdr *buf);
struct uip_udp_conn *uip_nextudpconn(struct uip_udp_conn *conn);

/* Defined in uip_udppoll.c *************************************************/

void uip_udppoll(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip_udpsend.c *************************************************/

void uip_udpsend(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip_udpinput.c ************************************************/

int uip_udpinput(struct uip_driver_s *dev);

/* Defined in uip_udpcallback.c *********************************************/

uint16_t uip_udpcallback(struct uip_driver_s *dev,
                         struct uip_udp_conn *conn, uint16_t flags);
#endif /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_ICMP
/* Defined in uip_icmpinput.c ***********************************************/

void uip_icmpinput(struct uip_driver_s *dev);

#ifdef CONFIG_NET_ICMP_PING
/* Defined in uip_icmpoll.c *************************************************/

void uip_icmppoll(struct uip_driver_s *dev);

/* Defined in uip_icmsend.c *************************************************/

void uip_icmpsend(struct uip_driver_s *dev, uip_ipaddr_t *destaddr);

#endif /* CONFIG_NET_ICMP_PING */
#endif /* CONFIG_NET_ICMP */

#ifdef CONFIG_NET_IGMP
/* Defined in uip_igmpinit.c ************************************************/

void uip_igmpinit(void);

/* Defined in uip_igmpinput.c ***********************************************/

void uip_igmpinput(struct uip_driver_s *dev);

/* Defined in uip_igmpgroup.c ***********************************************/

void uip_grpinit(void);
FAR struct igmp_group_s *uip_grpalloc(FAR struct uip_driver_s *dev,
                                      FAR const uip_ipaddr_t *addr);
FAR struct igmp_group_s *uip_grpfind(FAR struct uip_driver_s *dev,
                                     FAR const uip_ipaddr_t *addr);
FAR struct igmp_group_s *uip_grpallocfind(FAR struct uip_driver_s *dev,
                                          FAR const uip_ipaddr_t *addr);
void uip_grpfree(FAR struct uip_driver_s *dev,
                 FAR struct igmp_group_s *group);

/* Defined in uip_igmpmsg.c **************************************************/

void uip_igmpschedmsg(FAR struct igmp_group_s *group, uint8_t msgid);
void uip_igmpwaitmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/* Defined in uip_igmppoll.c *************************************************/

void uip_igmppoll(FAR struct uip_driver_s *dev);

/* Defined in up_igmpsend.c **************************************************/

void uip_igmpsend(FAR struct uip_driver_s *dev, FAR struct igmp_group_s *group,
                  FAR uip_ipaddr_t *dest);

/* Defined in uip_igmptimer.c ************************************************/

int uip_decisec2tick(int decisecs);
void uip_igmpstartticks(FAR struct igmp_group_s *group, int ticks);
void uip_igmpstarttimer(FAR struct igmp_group_s *group, uint8_t decisecs);
bool uip_igmpcmptimer(FAR struct igmp_group_s *group, int maxticks);

/* Defined in uip_mcastmac ***************************************************/

void uip_addmcastmac(FAR struct uip_driver_s *dev, FAR uip_ipaddr_t *ip);
void uip_removemcastmac(FAR struct uip_driver_s *dev, FAR uip_ipaddr_t *ip);

#endif /* CONFIG_NET_IGMP */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* _NET_UIP_UIP_INTERNAL_H */
