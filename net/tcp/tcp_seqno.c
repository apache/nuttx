/****************************************************************************
 * net/tcp/tcp_seqno.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Large parts of this file were leveraged from uIP logic:
 *
 *   Copyright (c) 2001-2003, Adam Dunkels.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <crypto/md5.h>
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>

#include <nuttx/clock.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These fields are used to generate initial TCP sequence numbers */

#ifdef CONFIG_NET_TCP_ISN_RFC6528
/* RFC 6528, Section 3: Key lengths of 128 bits should be adequate. */

static uint32_t g_tcp_isnkey[4];
#else
static uint32_t g_tcpsequence;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_isn_rfc6528
 *
 * Description:
 *   Calculate the initial sequence number described in RFC 6528.
 *   ISN = M + F(localip, localport, remoteip, remoteport, secretkey)
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_ISN_RFC6528
static uint32_t tcp_isn_rfc6528(FAR struct tcp_conn_s *conn)
{
  const size_t addrlen = net_ip_domain_select(conn->domain,
                                  sizeof(in_addr_t), sizeof(net_ipv6addr_t));
  MD5_CTX ctx;
  uint32_t digest[MD5_DIGEST_LENGTH / 4];
  uint32_t m;

  /* Make sure we have a secret key */

  if (g_tcp_isnkey[0] == 0)
    {
      arc4random_buf(g_tcp_isnkey, sizeof(g_tcp_isnkey));
    }

  /* M is the 4 microsecond timer */

  m = TICK2USEC(clock_systime_ticks()) / 4;

  /* F() is suggested to be MD5 */

  md5init(&ctx);

  /* Calculate F(localip, localport, remoteip, remoteport, secretkey) */

  md5update(&ctx, net_ip_binding_laddr(&conn->u, conn->domain), addrlen);
  md5update(&ctx, &conn->lport, sizeof(conn->lport));
  md5update(&ctx, net_ip_binding_raddr(&conn->u, conn->domain), addrlen);
  md5update(&ctx, &conn->rport, sizeof(conn->rport));
  md5update(&ctx, g_tcp_isnkey, sizeof(g_tcp_isnkey));

  md5final((FAR uint8_t *)digest, &ctx);

  /* ISN = M + F(localip, localport, remoteip, remoteport, secretkey) */

  return m + digest[0];
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_setsequence
 *
 * Description:
 *   Set the TCP/IP sequence number
 *
 * Assumptions:
 *   This function must be called with the network locked if seqno refers
 *   to a shared, global resource.
 *
 ****************************************************************************/

void tcp_setsequence(FAR uint8_t *seqno, uint32_t value)
{
  /* Copy the sequence number in network (big-endian) order */

  *seqno++ =  value >> 24;
  *seqno++ = (value >> 16) & 0xff;
  *seqno++ = (value >>  8) & 0xff;
  *seqno   =  value        & 0xff;
}

/****************************************************************************
 * Name: tcp_getsequence
 *
 * Description:
 *   Get the TCP/IP sequence number
 *
 * Assumptions:
 *   This function must be called with the network locked if seqno refers
 *   to a shared, global resource.
 *
 ****************************************************************************/

uint32_t tcp_getsequence(FAR uint8_t *seqno)
{
  uint32_t value;

  /* Combine the sequence number from network (big-endian) order */

  value = (uint32_t)seqno[0] << 24 |
          (uint32_t)seqno[1] << 16 |
          (uint32_t)seqno[2] <<  8 |
          (uint32_t)seqno[3];
  return value;
}

/****************************************************************************
 * Name: tcp_addsequence
 *
 * Description:
 *   Add the length to get the next TCP sequence number.
 *
 * Assumptions:
 *   This function must be called with the network locked if seqno refers
 *   to a shared, global resource.
 *
 ****************************************************************************/

uint32_t tcp_addsequence(FAR uint8_t *seqno, uint16_t len)
{
  return tcp_getsequence(seqno) + (uint32_t)len;
}

/****************************************************************************
 * Name: tcp_initsequence
 *
 * Description:
 *   Set the (initial) the TCP/IP sequence number when a TCP connection is
 *   established.
 *
 * Assumptions:
 *   This function must be called with the network locked if seqno refers
 *   to a shared, global resource.
 *
 ****************************************************************************/

void tcp_initsequence(FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_TCP_ISN_RFC6528
  tcp_setsequence(conn->sndseq, tcp_isn_rfc6528(conn));
#else
  /* If g_tcpsequence is already initialized, just copy it */

  if (g_tcpsequence == 0)
    {
      /* Get a random TCP sequence number */

      arc4random_buf(&g_tcpsequence, sizeof(uint32_t));

      /* Use about half of allowed values */

      g_tcpsequence = g_tcpsequence % 2000000000;

      /* If the random value is "small" increase it */

      if (g_tcpsequence < 1000000000)
        {
          g_tcpsequence += 1000000000;
        }
    }

  tcp_setsequence(conn->sndseq, g_tcpsequence);
#endif
}

/****************************************************************************
 * Name: tcp_nextsequence
 *
 * Description:
 *   Increment the TCP/IP sequence number
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void tcp_nextsequence(void)
{
#ifndef CONFIG_NET_TCP_ISN_RFC6528
  g_tcpsequence++;
#endif
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
