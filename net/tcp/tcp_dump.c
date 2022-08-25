/****************************************************************************
 * net/tcp/tcp_dump.c
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

#include <inttypes.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "tcp/tcp.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_event_handler_dump
 *
 * Description:
 *  Dump the TCP event handler related variables
 *
 ****************************************************************************/

void tcp_event_handler_dump(FAR struct net_driver_s *dev,
                            FAR void *pvpriv,
                            uint16_t flags,
                            FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  nerr("ERROR: conn->dev == NULL:"
       " dev=%p pvpriv=%p flags=0x%04x"
       " conn->dev=%p conn->flags=0x%04x tcpstateflags=0x%02x crefs=%d"
       " isn=%" PRIu32 " sndseq=%" PRIu32
       " tx_unacked=%" PRId32 " sent=%" PRId32
       " conn=%p s_flags=0x%02x\n",
       dev, pvpriv, flags,
       conn->dev, conn->flags, conn->tcpstateflags, conn->crefs,
       conn->isn, tcp_getsequence(conn->sndseq),
       (uint32_t)conn->tx_unacked, conn->sent,
       conn, conn->sconn.s_flags);
#else
  nerr("ERROR: conn->dev == NULL:"
       " dev=%p pvpriv=%p flags=0x%04x"
       " conn->dev=%p conn->flags=0x%04x tcpstateflags=0x%02x crefs=%d"
       " sndseq=%" PRIu32
       " tx_unacked=%" PRId32
       " conn=%p s_flags=0x%02x\n",
       dev, pvpriv, flags,
       conn->dev, conn->flags, conn->tcpstateflags, conn->crefs,
       tcp_getsequence(conn->sndseq),
       (uint32_t)conn->tx_unacked,
       conn, conn->sconn.s_flags);
#endif
}

/****************************************************************************
 * Name: tcp_wrbuffer_dump
 *
 * Description:
 *  Dump the contents of a write buffer
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS

void tcp_wrbuffer_dump(FAR const char *msg, FAR struct tcp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset)
{
  syslog(LOG_DEBUG, "%s: wrb=%p segno=%" PRIu32 " sent=%d nrtx=%d\n",
         msg, wrb, TCP_WBSEQNO(wrb), TCP_WBSENT(wrb), TCP_WBNRTX(wrb));
  iob_dump("I/O Buffer Chain", TCP_WBIOB(wrb), len, offset);
}

#endif

#endif /* CONFIG_DEBUG_FEATURES */
