/************************************************************
 * uip-udpconn.c
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

struct uip_udp_conn *uip_udpalloc(void);
void uip_udpfree(struct uip_udp_conn *conn);

#ifdef CONFIG_NET_IPv6
int uip_udpbind(struct uip_udp_conn *conn, const struct sockaddr_in6 *addr);
#else
int uip_udpbind(struct uip_udp_conn *conn, const struct sockaddr_in *addr);
#endif

#ifdef CONFIG_NET_IPv6
int uip_udpconnect(struct uip_udp_conn *conn, const struct sockaddr_in6 *addr )
#else
int uip_udpbind(struct uip_udp_conn *conn, const struct sockaddr_in *addr)
#endif
{
  uint16 ipaddr[2];

  if (pdhcpc->state == STATE_INITIAL)
    {
      uip_ipaddr(ipaddr, 0,0,0,0);
      uip_sethostaddr(ipaddr);
    }
  return OK;
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
      if (uip_udp_conns[c].lport == 0)
        {
          conn = &uip_udp_conns[c];
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

#endif /* CONFIG_NET */
