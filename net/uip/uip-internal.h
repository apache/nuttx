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

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/types.h>
#include <errno.h>
#include <arch/irq.h>
#include <net/uip/uip.h>

/****************************************************************************
 * Public Macro Definitions
 ****************************************************************************/

/* TCP definitions */

#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10
#define TCP_URG 0x20
#define TCP_CTL 0x3f

#define TCP_OPT_END     0   /* End of TCP options list */
#define TCP_OPT_NOOP    1   /* "No-operation" TCP option */
#define TCP_OPT_MSS     2   /* Maximum segment size TCP option */

#define TCP_OPT_MSS_LEN 4   /* Length of TCP MSS option. */

/* ICMP/ICMP6 definitions */

#define ICMP_ECHO_REPLY 0
#define ICMP_ECHO       8

#define ICMP6_ECHO_REPLY             129
#define ICMP6_ECHO                   128
#define ICMP6_NEIGHBOR_SOLICITATION  135
#define ICMP6_NEIGHBOR_ADVERTISEMENT 136

#define ICMP6_FLAG_S (1 << 6)

#define ICMP6_OPTION_SOURCE_LINK_ADDRESS 1
#define ICMP6_OPTION_TARGET_LINK_ADDRESS 2

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uip_ipaddr_t all_ones_addr;
extern const uip_ipaddr_t all_zeroes_addr;

/* Increasing number used for the IP ID field. */

extern uint16 g_ipid;

/* Reassembly timer (units: deci-seconds) */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
extern uint8 uip_reasstmr;
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

/* Defined in uip_tcpconn.c *************************************************/

EXTERN void uip_tcpinit(void);
EXTERN struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf);
EXTERN struct uip_conn *uip_nexttcpconn(struct uip_conn *conn);
EXTERN struct uip_conn *uip_tcplistener(uint16 portno);
EXTERN struct uip_conn *uip_tcpaccept(struct uip_tcpip_hdr *buf);
EXTERN void uip_tcpnextsequence(void);

/* Defined in uip-tcppoll.c *************************************************/

EXTERN void uip_tcppoll(struct uip_driver_s *dev, struct uip_conn *conn);

/* Defined in uip-udptimer.c ************************************************/

EXTERN void uip_tcptimer(struct uip_driver_s *dev, struct uip_conn *conn, int hsec);

/* Defined in uip_listen.c **************************************************/

EXTERN void uip_listeninit(void);
EXTERN boolean uip_islistener(uint16 port);
EXTERN int uip_accept(struct uip_conn *conn, uint16 portno);

/* Defined in uip-tcpsend.c *************************************************/

EXTERN void uip_tcpsend(struct uip_driver_s *dev, struct uip_conn *conn,
                        uint8 flags, uint16 len);
EXTERN void uip_tcpreset(struct uip_driver_s *dev);
EXTERN void uip_tcpack(struct uip_driver_s *dev, struct uip_conn *conn,
                       uint8 ack);

/* Defined in uip-tcpappsend.c **********************************************/

EXTERN void uip_tcpappsend(struct uip_driver_s *dev, struct uip_conn *conn,
                           uint8 result);
EXTERN void uip_tcprexmit(struct uip_driver_s *dev, struct uip_conn *conn,
                          uint8 result);

/* Defined in uip-tcpinput.c ************************************************/

EXTERN void uip_tcpinput(struct uip_driver_s *dev);

/* Defined in uip_uipcallback.c *********************************************/

EXTERN void uip_tcpcallback(struct uip_driver_s *dev);

#ifdef CONFIG_NET_UDP
/* Defined in uip_udpconn.c *************************************************/

EXTERN void uip_udpinit(void);
EXTERN struct uip_udp_conn *uip_udpactive(struct uip_udpip_hdr *buf);
EXTERN struct uip_udp_conn *uip_nextudpconn(struct uip_udp_conn *conn);

/* Defined in uip-udppoll.c *************************************************/

EXTERN void uip_udppoll(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip-udpsend.c *************************************************/

EXTERN void uip_udpsend(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip-udpinput.c ************************************************/

EXTERN void uip_udpinput(struct uip_driver_s *dev);

/* Defined in uip_uipcallback.c *********************************************/

EXTERN void uip_udpcallback(struct uip_driver_s *dev);
#endif /* CONFIG_NET_UDP */

/* Defined in uip-icmpinput.c ***********************************************/

EXTERN void uip_icmpinput(struct uip_driver_s *dev);

/* UIP logging **************************************************************/

/* This function must be provided by the application if CONFIG_NET_LOGGING
 * is defined.
 */

#ifdef CONFIG_NET_LOGGING
EXTERN void uip_log(char *msg);
#else
# define uip_log(m)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* __UIP_INTERNAL_H */
