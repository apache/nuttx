/************************************************************
 * uip-udpconn.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <arch/irq.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/************************************************************
 * Private Data
 ************************************************************/

/* The array containing all uIP UDP connections. */

struct uip_udp_conn uip_udp_conns[UIP_UDP_CONNS];

/* Last port used by a UDP connection connection. */

static uint16 g_last_udp_port;

/************************************************************
 * Private Functions
 ************************************************************/

#ifdef CONFIG_NET_UDP
struct uip_udp_conn *uip_find_udp_conn( uint16 portno )
{
  struct uip_udp_conn *conn;
  int i;

  for (i = 0; i < UIP_UDP_CONNS; i++)
    {
      if (uip_udp_conns[i].lport == htons(g_last_udp_port))
        {
          return conn;
        }
    }

  return NULL;
}
#endif   /* CONFIG_NET_UDP */

/************************************************************
 * Public Functions
 ************************************************************/

/****************************************************************************
 * Name: uip_udpinit()
 *
 * Description:
 *   Initialize the UDP connection structures.  Called once and only from
 *   the UIP layer.
 *
 ****************************************************************************/

void uip_udpinit(void)
{
  int i;
  for (i = 0; i < UIP_UDP_CONNS; i++)
    {
      uip_udp_conns[i].lport = 0;
    }

  g_last_udp_port = 1024;
}

/****************************************************************************
 * Name: uip_udpalloc()
 *
 * Description:
 *   Find a free UDP connection structure and allocate it for use.  This is
 *   normally something done by the implementation of the socket() API.
 *
 ****************************************************************************/

struct uip_udp_conn *uip_udpalloc(void)
{
#warning "Need to implement allocation logic"
  return NULL;
}

/****************************************************************************
 * Name: uip_udpfree()
 *
 * Description:
 *   Free a UDP connection structure that is no longer in use. This should be
 *   done by the implementation of close()
 *
 ****************************************************************************/

void uip_udpfree(struct uip_udp_conn *conn)
{
#warning "Need to implement release logic"
}

/****************************************************************************
 * Name: uip_udpactive()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used withi the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level
 *
 ****************************************************************************/

struct uip_udp_conn *uip_udpactive(struct uip_udpip_hdr *buf)
{
  struct uip_udp_conn *conn;
  for (conn = &uip_udp_conns[0]; conn < &uip_udp_conns[UIP_UDP_CONNS]; conn++)
    {
      /* If the local UDP port is non-zero, the connection is considered
       * to be used. If so, the local port number is checked against the
       * destination port number in the received packet. If the two port
       * numbers match, the remote port number is checked if the
       * connection is bound to a remote port. Finally, if the
       * connection is bound to a remote IP address, the source IP
       * address of the packet is checked.
       */

      if (conn->lport != 0 && buf->destport == conn->lport &&
          (conn->rport == 0 || buf->srcport == conn->rport) &&
            (uip_ipaddr_cmp(conn->ripaddr, all_zeroes_addr) ||
             uip_ipaddr_cmp(conn->ripaddr, all_ones_addr) ||
             uip_ipaddr_cmp(buf->srcipaddr, conn->ripaddr)))
        {
          /* Matching connection found.. return a reference to it */

          return conn;
        }
    }

  /* No match found */

  return NULL;
}

/****************************************************************************
 * Name: uip_udppoll()
 *
 * Description:
 *   Periodic processing for a UDP connection identified by its number.
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

void uip_udppoll(unsigned int conn)
{
  uip_udp_conn = &uip_udp_conns[conn];
  uip_interrupt(UIP_UDP_TIMER);
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
int uip_udpbind(struct uip_udp_conn *conn, const struct sockaddr_in6 *addr)
#else
int uip_udpbind(struct uip_udp_conn *conn, const struct sockaddr_in *addr)
#endif
{
#warning "Need to implement bind logic"
  return -ENOSYS;
}

/* Set up a new UDP connection.
 *
 * This function sets up a new UDP connection. The function will
 * automatically allocate an unused local port for the new
 * connection. However, another port can be chosen by using the
 * uip_udp_bind() call, after the uip_udp_new() function has been
 * called.
 *
 * Example:
 *
 *   uip_ipaddr_t addr;
 *   struct uip_udp_conn *c;
 * 
 *   uip_ipaddr(&addr, 192,168,2,1);
 *   c = uip_udp_new(&addr, HTONS(12345));
 *   if(c != NULL) {
 *     uip_udp_bind(c, HTONS(12344));
 *   }
 *
 * ripaddr The IP address of the remote host.
 *
 * rport The remote port number in network byte order.
 *
 * Return:  The uip_udp_conn structure for the new connection or NULL
 * if no connection could be allocated.
 */

struct uip_udp_conn *uip_udp_new(uip_ipaddr_t *ripaddr, uint16 rport)
{
  struct uip_udp_conn *conn;
  int i;

  /* Find an unused local port number.  Loop until we find a valid listen port
   * number that is not being used by any other connection.
   */

  do
    {
      /* Guess that the next available port number will be the one after
       * the last port number assigned.
       */

      ++g_last_udp_port;

      /* Make sure that the port number is within range */
      if (g_last_udp_port >= 32000)
        {
          g_last_udp_port = 4096;
        }
    }
  while (uip_find_udp_conn(g_last_udp_port));

  /* Now find an available UDP connection structure */

  conn = 0;
  for (i = 0; i < UIP_UDP_CONNS; i++)
    {
      if (uip_udp_conns[i].lport == 0)
        {
          conn = &uip_udp_conns[i];
          break;
        }
    }

  /* Return an error if no connection is available */

  if (conn == 0)
    {
      return 0;
    }

  /* Initialize and return the connection structure, bind it to the port number */

  conn->lport = HTONS(g_last_udp_port);
  conn->rport = rport;

  if (ripaddr == NULL)
    {
      memset(conn->ripaddr, 0, sizeof(uip_ipaddr_t));
    }
  else
    {
      uip_ipaddr_copy(&conn->ripaddr, ripaddr);
    }

  conn->ttl = UIP_TTL;

  return conn;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
