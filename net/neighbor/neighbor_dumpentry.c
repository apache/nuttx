/****************************************************************************
 * net/neighbor/neighbor_dumpentry.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "neighbor/neighbor.h"

#ifdef CONFIG_DEBUG_NET_INFO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/******************************************************************************
 * Name: neighbor_dump_address
 *
 * Description:
 *   Dump a data buffer to the SYSLOG.
 *
 * Input Parameters:
 *   buffer - The buffer to be dumped
 *   buflen - The length of the buffer in bytes.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void neighbor_dump_address(FAR const uint8_t *buffer, unsigned int buflen)
{
  char outbuf[16*3 + 9]; /* 6-byte header header + 16 hex bytes +
                          * 2 space separator + NUL termination */
  FAR char *ptr;
  unsigned int i;
  unsigned int j;
  unsigned int maxj;

  for (i = 0; i < buflen; i += 16)
    {
      if (i == 0)
        {
          ninfo("  at: ");
        }
      else
        {
          ninfo("      ");
        }

      maxj = 16;
      if (i + maxj > buflen)
        {
          maxj = buflen - i;
        }

      ptr = outbuf;
      for (j = 0; j < maxj; j++)
        {
          if (j == 8)
            {
              *ptr++ = ' ';
              *ptr++ = ' ';
            }

          sprintf(ptr, "%02x ", *buffer++);
          ptr += 3;
        }

      *ptr = '\0';
      wlinfo("  %s\n", ptr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_dumpentry
 *
 * Description:
 *   Dump the conents of an entry Neighbor Table.
 *
 * Input Parameters:
 *   msg      - Message to print with the entry
 *   neighbor - The table entry to dump
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void neighbor_dumpentry(FAR const char *msg,
                        FAR struct neighbor_entry *neighbor)
{
  ninfo("%s: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        msg,
        ntohs(neighbor->ne_ipaddr[0]), ntohs(neighbor->ne_ipaddr[1]),
        ntohs(neighbor->ne_ipaddr[2]), ntohs(neighbor->ne_ipaddr[3]),
        ntohs(neighbor->ne_ipaddr[4]), ntohs(neighbor->ne_ipaddr[5]),
        ntohs(neighbor->ne_ipaddr[6]), ntohs(neighbor->ne_ipaddr[7]));

#ifdef CONFIG_NET_ETHERNET
#ifdef CONFIG_NET_6LOWPAN
  if (neighbor->ne_addr.na_lltype == NET_LL_ETHERNET)
#endif
    {
      neighbor_dump_address(neighbor->ne_addr.u.na_ethernet.ether_addr_octet,
                            neighbor->ne_addr.na_llsize);
    }
#endif

#ifdef CONFIG_NET_6LOWPAN
#ifdef CONFIG_NET_ETHERNET
  else
#endif
    {
      neighbor_dump_address(neighbor->ne_addr.u.na_sixlowpan.nm_addr,
                            neighbor->ne_addr.na_llsize);
    }
#endif /* CONFIG_NET_6LOWPAN */
}

/****************************************************************************
 * Name: neighbor_dumpipaddr
 *
 * Description:
 *   Dump an IP address.
 *
 * Input Parameters:
 *   msg    - Message to print with the entry
 *   ipaddr - The IP address to dump
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void neighbor_dumpipaddr(FAR const char *msg,
                         const net_ipv6addr_t ipaddr)
{
  ninfo("%s: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        msg,
        ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]),
        ntohs(ipaddr[3]), ntohs(ipaddr[4]), ntohs(ipaddr[5]),
        ntohs(ipaddr[6]), ntohs(ipaddr[7]));
}

#endif /* CONFIG_DEBUG_NET_INFO */
