/****************************************************************************
 * uip_tcpbind.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

/****************************************************************************
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/types.h>
#include <string.h>
#include <arch/irq.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_tcp_sequence[] is used to generate TCP sequence numbers */

uint8 g_tcp_sequence[4];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all uIP connections. */

static struct uip_conn g_tcp_connections[UIP_CONNS];

/* Last port used by a TCP connection connection. */

static uint16 g_last_tcp_port;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Given a port number, find the socket bound to the port number.
 * Primary use: to determine if a port number is available.
 */

static struct uip_conn *uip_find_conn(uint16 portno)
{
  struct uip_conn *conn;
  int i;

  /* Check if this port number is already in use, and if so try to find
   * another one.
   */

  for (i = 0; i < UIP_CONNS; i++)
    {
      conn = &g_tcp_connections[i];
      if (conn->tcpstateflags != UIP_CLOSED && conn->lport == htons(g_last_tcp_port))
        {
          /* The portnumber is in use */

          return conn;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpinit()
 *
 * Description:
 *   Initialize the TCP/IP connection structures.  Called only from the UIP
 *   layer.
 *
 ****************************************************************************/

void uip_tcpinit(void)
{
  int i;
  for (i = 0; i < UIP_CONNS; i++)
    {
      g_tcp_connections[i].tcpstateflags = UIP_CLOSED;
    }

  g_last_tcp_port = 1024;
}

/****************************************************************************
 * Name: uip_tcpalloc()
 *
 * Description:
 *   Find a free TCP/IP connection structure and allocate it
 *   for use.  This is normally something done by the implementation of the
 *   socket() API but is also called from the interrupt level when a TCP
 *   packet is received while "listening"
 *
 ****************************************************************************/

struct uip_conn *uip_tcpalloc(void)
{
#if 0 /* Revisit */
  struct uip_conn *oldest = NULL;
#endif
  irqstate_t flags;
  unsigned int i;

  /* Because this routine is called from both interrupt level and
   * and from user level, we have not option but to disable interrupts
   * while accessing g_tcp_connections[];
   */

  flags = irqsave();

  /* Check if there are any available connections. */

  for (i = 0; i < UIP_CONNS; i++)
    {
      /* First, check if any connections structures are marked as
       * CLOSED in the table of pre-allocated connection structures.
       */

      if (g_tcp_connections[i].tcpstateflags == UIP_CLOSED)
        {
          /* We found an unused structure. Mark as allocated, but not
           * initialized.
           */

          memset(&g_tcp_connections[i], 0, sizeof(struct uip_conn));
          g_tcp_connections[i].tcpstateflags = UIP_ALLOCATED;

          irqrestore(flags);
          return &g_tcp_connections[i];
        }

#if 0 /* Revisit */
      /* As a fallback, check for connection structures in the TIME_WAIT
       * state.  If no CLOSED connections are found, then take the oldest
       */

      if (g_tcp_connections[i].tcpstateflags == UIP_TIME_WAIT)
        {
          if (!oldest || g_tcp_connections[i].timer > oldest->timer)
            {
              oldest = &g_tcp_connections[i];
            }
        }
    }
  return oldest;
#else
    }

  irqrestore(flags);
  return NULL;
#endif
}

/****************************************************************************
 * Name: uip_tcpfree()
 *
 * Description:
 *   Free a connection structure that is no longer in use. This should be
 *   done by the implementation of close()
 *
 ****************************************************************************/

void uip_tcpfree(struct uip_conn *conn)
{
  /* this action is atomic and should require no special protetion */

  conn->tcpstateflags = UIP_CLOSED;
}

/****************************************************************************
 * Name: uip_tcpactive()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used withi the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level
 *
 ****************************************************************************/

struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf)
{
  struct uip_conn *conn;
  for (conn = g_tcp_connections; conn <= &g_tcp_connections[UIP_CONNS - 1]; conn++)
    {
      /* Find an open connection matching the tcp input */

      if (conn->tcpstateflags != UIP_CLOSED &&
           buf->destport == conn->lport && buf->srcport == conn->rport &&
           uip_ipaddr_cmp(buf->srcipaddr, conn->ripaddr))
        {
          return conn;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: uip_tcppoll()
 *
 * Description:
 *   Periodic processing for a connection identified by its number.
 *   This function does the necessary periodic processing (timers,
 *   polling) for a uIP TCP conneciton, and should be called by the UIP
 *   device driver when the periodic uIP timer goes off. It should be
 *   called for every connection, regardless of whether they are open of
 *   closed.
 *
 * Assumptions:
 *   This function is called from the CAN device driver may be called from
 *   the timer interrupt/watchdog handle level.
 *
 ****************************************************************************/

void uip_tcppoll(unsigned int conn)
{
  uip_conn = &g_tcp_connections[conn];
  uip_interrupt(UIP_TIMER);
}

/****************************************************************************
 * Name: uip_tcpactive()
 *
 * Description:
 *   Increment the TCP/IP sequence number
 *
 * Assumptions:
 *   This function is called from the interrupt level
 *
 ****************************************************************************/

void uip_tcpnextsequence(void)
{
  /* This inplements a byte-by-byte big-endian increment */

  if (++g_tcp_sequence[3] == 0)
    {
      if (++g_tcp_sequence[2] == 0)
        {
          if (++g_tcp_sequence[1] == 0)
            {
              ++g_tcp_sequence[0];
            }
        }
    }
}

/****************************************************************************
 * Name: uip_tcpbind()
 *
 * Description:
 *   This function implements the UIP specific parts of the standard TCP
 *   bind() operation.
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in6 *addr)
#else
int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in *addr)
#endif
{
#warning "Need to implement bind logic"
  return ERROR;
}

/****************************************************************************
 * Name: uip_tcpbind()
 *
 * Description:
 *   This function implements the UIP specific parts of the standard
 *   TCP connect() operation:  It connects to a remote host using TCP.
 *
 *   This function is used to start a new connection to the specified
 *   port on the specied host. It uses the connection structure that was
 *   allocated by a preceding socket() call.  It sets the connection to
 *   the SYN_SENT state and sets the retransmission timer to 0. This will
 *   cause a TCP SYN segment to be sent out the next time this connection
 *   is periodically processed, which usually is done within 0.5 seconds
 *   after the call to uip_tcpconnect().
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in6 *addr )
#else
int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in *addr )
#endif
{
  uint16 port;
  int i;

  /* If the TCP port has not alread been bound to a local port, then select
   * one now.
   */

  port = ntohs(conn->lport);
  if (port == 0)
    {
      /* No local port assigned. Loop until we find a valid listen port number\
       * that is not being used by any other connection.
       */

      do
        {
          /* Guess that the next available port number will be the one after
           * the last port number assigned.
           */
#warning "This need protection from other threads and from interrupts"
          port = ++g_last_tcp_port;

          /* Make sure that the port number is within range */

          if (g_last_tcp_port >= 32000)
            {
              g_last_tcp_port = 4096;
            }
        }
      while (uip_find_conn(g_last_tcp_port));
    }

  /* Initialize and return the connection structure, bind it to the port number */

  conn->tcpstateflags = UIP_SYN_SENT;

  conn->snd_nxt[0] = g_tcp_sequence[0];
  conn->snd_nxt[1] = g_tcp_sequence[1];
  conn->snd_nxt[2] = g_tcp_sequence[2];
  conn->snd_nxt[3] = g_tcp_sequence[3];

  conn->initialmss = conn->mss = UIP_TCP_MSS;

  conn->len        = 1;    /* TCP length of the SYN is one. */
  conn->nrtx       = 0;
  conn->timer      = 1;    /* Send the SYN next time around. */
  conn->rto        = UIP_RTO;
  conn->sa         = 0;
  conn->sv         = 16;   /* Initial value of the RTT variance. */
  conn->lport      = htons(port);

  /* The sockaddr port is 16 bits and already in network order */

  conn->rport = addr->sin_port;

  /* The sockaddr address is 32-bits in network order. */

  uip_ipaddr_copy(&conn->ripaddr, addr->sin_addr.s_addr);
  return OK;
}

#endif /* CONFIG_NET */
