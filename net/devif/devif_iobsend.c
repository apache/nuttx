/****************************************************************************
 * net/devif/devif_iobsend.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_iob_send
 *
 * Description:
 *   Called from socket logic in response to a xmit or poll request from the
 *   the network interface driver.
 *
 *   This is identical to calling devif_send() except that the data is
 *   in an I/O buffer chain, rather than a flat buffer.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void devif_iob_send(FAR struct net_driver_s *dev, FAR struct iob_s *iob,
                    unsigned int len, unsigned int offset,
                    unsigned int target_offset)
{
#ifndef CONFIG_NET_IPFRAG
  unsigned int limit = NETDEV_PKTSIZE(dev) -
                       NET_LL_HDRLEN(dev) - target_offset;
#endif
  int ret;

#ifndef CONFIG_NET_IPFRAG
  if (dev == NULL || len == 0 || len > limit)
#else
  if (dev == NULL || len == 0)
#endif
    {
      if (dev->d_iob == NULL)
        {
          iob_free_chain(iob);
        }

#ifndef CONFIG_NET_IPFRAG
      nerr("devif_iob_send error, %p, send len: %u, limit len: %u\n",
           dev, len, limit);
#else
      nerr("devif_iob_send error, %p, send len: %u\n", dev, len);
#endif
      return;
    }

  /* Append the send buffer after device buffer */

  if (dev->d_iob != NULL)
    {
      dev->d_sndlen = 0;

      if (len > iob_navail(false) * CONFIG_IOB_BUFSIZE)
        {
          return;
        }

      /* Clone the iob to target device buffer */

      ret = iob_clone_partial(iob, len, offset, dev->d_iob,
                              target_offset, false, false);
      if (ret != OK)
        {
          netdev_iob_release(dev);
          nerr("devif_iob_send error, not enough iob entries, "
              "send len: %u\n", len);
          return;
        }

      dev->d_sndlen = len;
    }
  else
    {
      /* Send the iob directly if no device buffer */

      dev->d_iob    = iob;
      dev->d_sndlen = len;
      dev->d_buf    = NETLLBUF;
    }

#ifdef CONFIG_NET_TCP_WRBUFFER_DUMP
  /* Dump the outgoing device buffer */

  lib_dumpbuffer("devif_iob_send", dev->d_appdata, len);
#endif
}

#endif /* CONFIG_MM_IOB */
