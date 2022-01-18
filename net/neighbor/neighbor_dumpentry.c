/****************************************************************************
 * net/neighbor/neighbor_dumpentry.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/neighbor.h>

#include "neighbor/neighbor.h"

#ifdef CONFIG_DEBUG_NET_INFO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

static void neighbor_dump_address(FAR const void *buf, unsigned int buflen)
{
  char outbuf[16 * 3 + 9]; /* 6-byte header header + 16 hex bytes +
                            * 2 space separator + NUL termination */
  FAR const uint8_t *buffer = buf;
  FAR char *ptr;
  unsigned int i;
  unsigned int j;
  unsigned int maxj;

  for (i = 0; i < buflen; i += 16)
    {
      if (i == 0)
        {
          sprintf(outbuf, "  at: ");
        }
      else
        {
          sprintf(outbuf, "      ");
        }

      maxj = 16;
      if (i + maxj > buflen)
        {
          maxj = buflen - i;
        }

      ptr = outbuf + 6;
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
      ninfo("%s\n", ptr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_dumpentry
 *
 * Description:
 *   Dump the contents of an entry Neighbor Table.
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
                        FAR struct neighbor_entry_s *neighbor)
{
  ninfo("%s: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        msg,
        NTOHS(neighbor->ne_ipaddr[0]), NTOHS(neighbor->ne_ipaddr[1]),
        NTOHS(neighbor->ne_ipaddr[2]), NTOHS(neighbor->ne_ipaddr[3]),
        NTOHS(neighbor->ne_ipaddr[4]), NTOHS(neighbor->ne_ipaddr[5]),
        NTOHS(neighbor->ne_ipaddr[6]), NTOHS(neighbor->ne_ipaddr[7]));

  neighbor_dump_address(&neighbor->ne_addr.u,
                        neighbor->ne_addr.na_llsize);
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
        NTOHS(ipaddr[0]), NTOHS(ipaddr[1]), NTOHS(ipaddr[2]),
        NTOHS(ipaddr[3]), NTOHS(ipaddr[4]), NTOHS(ipaddr[5]),
        NTOHS(ipaddr[6]), NTOHS(ipaddr[7]));
}

#endif /* CONFIG_DEBUG_NET_INFO */
