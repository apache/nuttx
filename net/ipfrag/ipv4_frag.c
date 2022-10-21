/****************************************************************************
 * net/ipfrag/ipv4_frag.c
 * Handling incoming IPv4 fragment input
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

#if defined(CONFIG_NET_IPv4) && defined (CONFIG_NET_IPFRAG)

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <debug.h>
#include <string.h>
#include <errno.h>
#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "ipfrag.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int32_t
ipv4_fragin_getinfo(FAR struct iob_s *iob,
                    FAR struct ip_fraglink_s *fraglink);
static uint32_t ipv4_fragin_reassemble(FAR struct ip_fragsnode_s *node);
static inline void
ipv4_fragout_buildipv4header(FAR struct ipv4_hdr_s *ref,
                             FAR struct ipv4_hdr_s *ipv4,
                             uint16_t len, uint16_t ipoff);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_fragin_getinfo
 *
 * Description:
 *   Polulate fragment information from the input ipv4 packet data.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int32_t
ipv4_fragin_getinfo(FAR struct iob_s *iob,
                    FAR struct ip_fraglink_s *fraglink)
{
  FAR struct ipv4_hdr_s *ipv4 = (FAR struct ipv4_hdr_s *)
                                (iob->io_data + iob->io_offset);
  uint16_t offset;

  fraglink->flink     = NULL;
  fraglink->fragsnode = NULL;
  fraglink->isipv4    = true;

  offset = (ipv4->ipoffset[0] << 8) + ipv4->ipoffset[1];
  fraglink->morefrags = offset & IP_FLAG_MOREFRAGS;
  fraglink->fragoff   = ((offset & 0x1fff) << 3);

  fraglink->fraglen   = (ipv4->len[0] << 8) + ipv4->len[1] - IPv4_HDRLEN;
  fraglink->ipid      = (ipv4->ipid[0] << 8) + ipv4->ipid[1];
  fraglink->frag      = iob;

  return OK;
}

/****************************************************************************
 * Name: ipv4_fragin_reassemble
 *
 * Description:
 *   Reassemble all ipv4 fragments to build an IP frame.
 *
 * Returned Value:
 *   The length of the reassembled IP frame
 *
 ****************************************************************************/

static uint32_t ipv4_fragin_reassemble(FAR struct ip_fragsnode_s *node)
{
  FAR struct iob_s *head;
  FAR struct ipv4_hdr_s *ipv4;
  FAR struct ip_fraglink_s *fraglink;

  /* Loop to walk through the fragment list and reassemble those fragments,
   * the fraglink list was ordered by fragment offset value
   */

  fraglink    = node->frags;
  node->frags = NULL;

  while (fraglink)
    {
      FAR struct ip_fraglink_s *linknext;
      FAR struct iob_s *iob = fraglink->frag;

      if (fraglink->fragoff != 0)
        {
          uint16_t iphdrlen;

          /* Get IPv4 header length from IPv4 header (it may carry some
           * IPv4 options)
           */

          ipv4 = (FAR struct ipv4_hdr_s *)(head->io_data + head->io_offset);
          iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

          /* Just modify the offset and length of all none zero fragments */

          iob->io_offset += iphdrlen;
          iob->io_len    -= iphdrlen;
          iob->io_pktlen -= iphdrlen;

          /* Concatenate this iob to the reassembly chain */

          iob_concat(head, iob);
        }
      else
        {
          /* Remember the head iob */

          head = iob;
        }

      linknext = fraglink->flink;
      kmm_free(fraglink);

      fraglink = linknext;
    }

  /* Remember the reassembled outgoing IP frame */

  node->outgoframe = head;

  /* Get pointer of the new IPv4 header */

  ipv4 = (FAR struct ipv4_hdr_s *)(head->io_data + head->io_offset);

  /* Update the length value in the IP Header */

  ipv4->len[0] = head->io_pktlen >> 8;
  ipv4->len[1] = head->io_pktlen & 0xff;

  /* Set ipoffset to zero */

  ipv4->ipoffset[0] = 0;
  ipv4->ipoffset[1] = 0;

  /* Calculate IP checksum. */

  ipv4->ipchksum    = 0;
  ipv4->ipchksum    = ~(ipv4_chksum(ipv4));

  return head->io_pktlen;
}

/****************************************************************************
 * Name: build_frag_ipv4_header
 *
 * Description:
 *   Build IPv4 header for an IPv4 fragment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
ipv4_fragout_buildipv4header(FAR struct ipv4_hdr_s *ref,
                             FAR struct ipv4_hdr_s *ipv4,
                             uint16_t len, uint16_t ipoff)
{
  if (ref != ipv4)
    {
      uint32_t iphdrlen = (ref->vhl & IPv4_HLMASK) << 2;
      memcpy(ipv4, ref, iphdrlen);
    }

  ipv4->len[0]      = len >> 8;
  ipv4->len[1]      = len & 0xff;

  ipv4->ipoffset[0] = ipoff >> 8;
  ipv4->ipoffset[1] = ipoff & 0xff;

  /* Calculate IP checksum. */

  ipv4->ipchksum    = 0;
  ipv4->ipchksum    = ~(ipv4_chksum(ipv4));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_fragin
 *
 * Description:
 *   Handling incoming IPv4 and IPv6 fragment input, the input data
 *   (dev->d_iob) can be an I/O buffer chain
 *
 * Input Parameters:
 *   dev    - The NIC device that the fragmented data comes from
 *
 * Returned Value:
 *   ENOMEM - No memory
 *   OK     - The input fragment is processed as expected
 *
 ****************************************************************************/

int32_t ipv4_fragin(FAR struct net_driver_s *dev)
{
  FAR struct ip_fragsnode_s *node;
  FAR struct ip_fraglink_s  *fraginfo;
  bool restartwdog;

  if (dev->d_len != dev->d_iob->io_pktlen)
    {
      nerr("ERROR: Parameters error.\n");
      return -EINVAL;
    }

  fraginfo = kmm_malloc(sizeof(struct ip_fraglink_s));
  if (fraginfo == NULL)
    {
      nerr("ERROR: Failed to allocate buffer.\n");
      return -ENOMEM;
    }

  /* Polulate fragment information from input packet data */

  ipv4_fragin_getinfo(dev->d_iob, fraginfo);

  nxsem_wait_uninterruptible(&g_ipfrag_mutex);

  /* Need to restart reassembly worker if the original linked list is empty */

  restartwdog = ip_fragin_enqueue(dev, fraginfo);

  node = fraginfo->fragsnode;

  if (node->verifyflag & IP_FRAGVERIFY_RECVDALLFRAGS)
    {
      /* Well, all fragments of an IP frame have been received, remove
       * node from link list first, then reassemble and dispatch to the
       * stack.
       */

      ip_frag_remnode(node);

      /* All fragments belonging to one IP frame have been separated
       * from the fragment processing module, unlocks mutex as soon
       * as possible
       */

      nxsem_post(&g_ipfrag_mutex);

      /* Reassemble fragments to one IP frame and set the resulting
       * IP frame to dev->d_iob
       */

      ipv4_fragin_reassemble(node);
      netdev_iob_replace(dev, node->outgoframe);

      /* Free the memory of node */

      kmm_free(node);

      return ipv4_input(dev);
    }

  nxsem_post(&g_ipfrag_mutex);

  if (restartwdog)
    {
      /* Restart the work queue for fragment processing */

      ip_frag_startwdog();
    }

  return OK;
}

/****************************************************************************
 * Name: ipv4_fragout
 *
 * Description:
 *   Execute the ipv4 fragment function. After this work is done, all
 *   fragments are maintained by dev->d_fragout. In order to reduce the
 *   cyclomatic complexity and facilitate maintenance, fragmentation is
 *   performed in two steps:
 *   1. Reconstruct I/O Buffer according to MTU, which will reserve
 *      the space for the L3 header;
 *   2. Fill the L3 header into the reserved space.
 *
 * Input Parameters:
 *   dev    - The NIC device
 *   mtu    - The MTU of current NIC
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 * Assumptions:
 *   Data length(dev->d_iob->io_pktlen) is grater than the MTU of
 *   current NIC
 *
 ****************************************************************************/

int32_t ipv4_fragout(FAR struct net_driver_s *dev, uint16_t mtu)
{
  uint32_t iter;
  uint32_t nfrags;
  uint16_t offset = 0;
  uint16_t hdrlen;
  FAR struct iob_s *frag;
  FAR struct ipv4_hdr_s *ref;
  struct iob_queue_s fragq =
    {
      NULL, NULL
    };

  /* Get the total length of L3 Header(if IPv4 options are present, then this
   * length includes the size of all the IPv4 options)
   */

  hdrlen = (IPv4BUF->vhl & IPv4_HLMASK) << 2;

  /* Reconstruct I/O Buffer according to MTU, which will reserve
   * the space for the L3 header
   */

  nfrags = ip_fragout_slice(dev->d_iob, PF_INET, mtu, hdrlen, &fragq);
  assert(nfrags > 1);
  netdev_iob_clear(dev);

  /* Fill the L3 header into the reserved space */

  for (iter = 0; iter < nfrags; iter++)
    {
      frag = iob_remove_queue(&fragq);

      if (iter == 0)
        {
          ref = (FAR struct ipv4_hdr_s *)(frag->io_data + frag->io_offset);

          /* Update the IPv4 header of the first fragment */

          ipv4_fragout_buildipv4header(ref, ref, frag->io_pktlen,
                                       IP_FLAG_MOREFRAGS);
        }
      else
        {
          uint16_t ipoff = (offset - iter * hdrlen) >> 3;

          if (iter < nfrags - 1)
            {
              ipoff |= IP_FLAG_MOREFRAGS;
            }

          /* Refer to the zero fragment ipv4 header to construct the ipv4
           * header of non-zero fragment
           */

          ipv4_fragout_buildipv4header(ref,
                  (FAR struct ipv4_hdr_s *)(frag->io_data + frag->io_offset),
                  frag->io_pktlen, ipoff);
        }

      /* Enqueue this fragment to dev->d_fragout */

      if (iob_tryadd_queue(frag, &dev->d_fragout) < 0)
        {
          goto fail;
        }

      offset += frag->io_pktlen;
    }

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv4.sent += nfrags - 1;
#endif

  netdev_txnotify_dev(dev);

  return OK;

fail:
  netdev_iob_release(dev);
  iob_free_chain(frag);
  iob_free_queue(&fragq);
  iob_free_queue(&dev->d_fragout);
  return -ENOMEM;
}

#endif /* CONFIG_NET_IPv4 && CONFIG_NET_IPFRAG */
