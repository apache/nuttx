/****************************************************************************
 * net/ipfrag/ipv6_frag.c
 * Handling incoming IPv6 fragment input
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
#if defined(CONFIG_NET_IPv6) && defined (CONFIG_NET_IPFRAG)

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
#include <nuttx/net/ipv6ext.h>

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "ipfrag.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The increasing number used for the IP ID field of IPv6 Fragment Header. */

static uint32_t g_ipv6id;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t ipv6_fragin_getinfo(FAR struct iob_s *iob,
                                   FAR struct ip_fraglink_s *fraglink);
static uint32_t ipv6_fragin_reassemble(FAR struct ip_fragsnode_s *node);
static inline void
ipv6_fragout_buildipv6header(FAR struct ipv6_hdr_s *ref,
                             FAR struct ipv6_hdr_s *ipv6,
                             uint16_t hdrlen, uint16_t datalen,
                             uint16_t nxthdroff, uint16_t nxtprot);
static inline void
ipv6_fragout_buildipv6fragheader(FAR struct ipv6_fragment_extension_s *frag,
                                 uint8_t nxthdr, uint16_t ipoff,
                                 uint32_t ipid);
static uint16_t ipv6_fragout_getunfraginfo(FAR struct iob_s *iob,
                                           uint16_t *hdroff,
                                           uint16_t *hdrtype);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_fragin_getinfo
 *
 * Description:
 *   Polulate fragment information from the input ipv6 packet data.
 *
 * Returned Value:
 *   OK    - Got fragment information.
 *   EINVAL - The input ipv6 packet is not a fragment.
 *
 ****************************************************************************/

static int32_t ipv6_fragin_getinfo(FAR struct iob_s *iob,
                                   FAR struct ip_fraglink_s *fraglink)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)
                                (iob->io_data + iob->io_offset);
  FAR struct ipv6_extension_s *exthdr;
  FAR uint8_t *payload;
  uint16_t     paylen;
  uint8_t      nxthdr;

  paylen  = ((uint16_t)ipv6->len[0] << 8) + (uint16_t)ipv6->len[1];
  payload = (FAR uint8_t *)(ipv6 + 1);
  exthdr  = (FAR struct ipv6_extension_s *)payload;
  nxthdr  = ipv6->proto;

  while (nxthdr != NEXT_FRAGMENT_EH && ipv6_exthdr(nxthdr))
    {
      uint16_t extlen;

      exthdr   = (FAR struct ipv6_extension_s *)payload;
      extlen   = EXTHDR_LEN((unsigned int)exthdr->len);
      payload += extlen;
      paylen  -= extlen;
      nxthdr   = exthdr->nxthdr;
    };

  if (nxthdr == NEXT_FRAGMENT_EH)
    {
      FAR struct ipv6_fragment_extension_s *fraghdr;

      fraghdr = (FAR struct ipv6_fragment_extension_s *)exthdr;

      /* Cut the size of fragment header, notice fragment header don't has a
       * length filed.
       */

      paylen -= EXTHDR_FRAG_LEN;

      fraglink->flink     = NULL;
      fraglink->fragsnode = NULL;

      fraglink->isipv4    = FALSE;
      fraglink->fragoff   = (fraghdr->msoffset << 8) + fraghdr->lsoffset;
      fraglink->morefrags = fraglink->fragoff & 0x1;
      fraglink->fragoff  &= 0xfff8;
      fraglink->fraglen   = paylen;
      fraglink->ipid      = NTOHL((*(uint16_t *)(&fraghdr->id[0]) << 16) +
                                  *(uint16_t *)(&fraghdr->id[2]));

      fraglink->frag      = iob;

      return OK;
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: ipv6_fragin_reassemble
 *
 * Description:
 *   Reassemble all ipv6 fragments to build an IP frame.
 *
 * Returned Value:
 *   The length of the reassembled IP frame
 *
 ****************************************************************************/

static uint32_t ipv6_fragin_reassemble(FAR struct ip_fragsnode_s *node)
{
  FAR struct iob_s      *head;
  FAR struct ipv6_hdr_s *ipv6;
  FAR struct ip_fraglink_s *fraglink;

  /* Loop to walk through the fragment list and reassemble those fragments,
   * the fraglink list was ordered by fragment offset value
   */

  fraglink = node->frags;
  node->frags = NULL;

  while (fraglink)
    {
      FAR uint8_t *payload;
      uint8_t      nxthdr;
      FAR struct iob_s *iob;
      FAR struct ip_fraglink_s *linknext;
      FAR struct ipv6_extension_s *exthdr;
      FAR struct ipv6_fragment_extension_s *fraghdr;

      iob     = fraglink->frag;
      ipv6    = (FAR struct ipv6_hdr_s *)(iob->io_data + iob->io_offset);
      payload = (FAR uint8_t *)(ipv6 + 1);
      exthdr  = (FAR struct ipv6_extension_s *)payload;
      nxthdr  = ipv6->proto;

      /* Find fragment header and the front header which is close to the
       * fragment header
       */

      while (nxthdr != NEXT_FRAGMENT_EH && ipv6_exthdr(nxthdr))
        {
          uint16_t extlen;

          exthdr   = (FAR struct ipv6_extension_s *)payload;
          extlen   = EXTHDR_LEN((unsigned int)exthdr->len);
          payload += extlen;
          nxthdr   = exthdr->nxthdr;
        };

      fraghdr  = (FAR struct ipv6_fragment_extension_s *)payload;

      /* Skip fragment header, notice fragment header don't has a length
       * filed
       */

      payload += EXTHDR_FRAG_LEN;

      if (fraglink->fragoff == 0)
        {
          /* This is the zero fragment, Set the front header's next header
           * filed to the next header value of the fragment header
           */

          if (ipv6->proto == NEXT_FRAGMENT_EH)
            {
              ipv6->proto = fraghdr->nxthdr;
            }
          else
            {
              exthdr->nxthdr = fraghdr->nxthdr;
            }

          /* Remove fragment header and fix up the data length */

          memmove(fraghdr, payload,
                  iob->io_len - (payload - (iob->io_data + iob->io_offset)));
          iob->io_len    -= EXTHDR_FRAG_LEN;
          iob->io_pktlen -= EXTHDR_FRAG_LEN;

          /* Remember the head iob */

          head = iob;
        }
      else
        {
          uint16_t new_off;

          /* Fix up the value of offset and length for this none zero
           * fragment
           */

          new_off         = payload - iob->io_data;
          iob->io_len    -= new_off - iob->io_offset;
          iob->io_pktlen -= new_off - iob->io_offset;
          iob->io_offset  = new_off;

          /* Concatenate this iob to the reassembly chain */

          iob_concat(head, iob);
        }

      linknext = fraglink->flink;
      kmm_free(fraglink);
      fraglink = linknext;
    }

  /* Remember the reassembled outgoing IP frame */

  node->outgoframe = head;

  /* Adjust the length value in the IP Header */

  ipv6 = (FAR struct ipv6_hdr_s *)(head->io_data + head->io_offset);
  ipv6->len[0] = (head->io_pktlen - IPv6_HDRLEN) >> 8;
  ipv6->len[1] = (head->io_pktlen - IPv6_HDRLEN) & 0xff;

  return head->io_pktlen;
}

/****************************************************************************
 * Name: ipv6_fragout_buildipv6header
 *
 * Description:
 *   Build IPv6 header for an IPv6 fragment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
ipv6_fragout_buildipv6header(FAR struct ipv6_hdr_s *ref,
                             FAR struct ipv6_hdr_s *ipv6,
                             uint16_t hdrlen, uint16_t datalen,
                             uint16_t nxthdroff, uint16_t nxtprot)
{
  if (ref != ipv6)
    {
      /* Copy unfragmentable header data from reference header */

      memcpy(ipv6, ref, hdrlen);
    }

  /* Update length filed */

  ipv6->len[0]      = datalen >> 8;
  ipv6->len[1]      = datalen & 0xff;

  /* If extension headers exist, update the Next Header field in the
   * last extension header of the unfragmentable part; Otherwise update
   * the Next Header field of the basic IPv6 header.
   */

  *((uint8_t *)ipv6 + nxthdroff) = nxtprot;
}

/****************************************************************************
 * Name: ipv6_fragout_buildipv6fragheader
 *
 * Description:
 *   Build IPv6 fragment extension header for an IPv6 fragment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
ipv6_fragout_buildipv6fragheader(FAR struct ipv6_fragment_extension_s *frag,
                                 uint8_t nxthdr, uint16_t ipoff,
                                 uint32_t ipid)
{
  frag->nxthdr = nxthdr;
  frag->reserved = 0;
  frag->msoffset = ipoff >> 8;
  frag->lsoffset = ipoff & 0xff;
  *(uint16_t *)&frag->id[0] = HTONL(ipid) & 0xffff;
  *(uint16_t *)&frag->id[2] = HTONL(ipid) >> 16;
}

/****************************************************************************
 * Name: ipv6_fragout_getunfraginfo
 *
 * Description:
 *   Get the length of Unfragmentable Part of the original ipv6 packet,
 *   remember the offset and value of nextheader in the last extension
 *   header of the unfragmentable part.
 *   Refer to rfc2460, section-4.1, section-4.5
 *
 * Input Parameters:
 *   iob    - Outgoing data waiting for fragment
 *   hdroff - The offset of the last next header position in the
 *              unfragmentable part
 *   hdrtype - The first header type in the fragmentable part
 *
 * Returned Value:
 *   Unfragmentable Part length
 *
 ****************************************************************************/

static uint16_t ipv6_fragout_getunfraginfo(FAR struct iob_s *iob,
                                           uint16_t *hdroff,
                                           uint16_t *hdrtype)
{
  uint32_t   iter = 0;
  bool       destopt = false;
  uint16_t   delta = sizeof(struct ipv6_hdr_s);
  uint16_t   unfraglen = delta;
  uint8_t    nxthdr;
  FAR struct ipv6_hdr_s *ipv6;
  FAR struct ipv6_extension_s *exthdr;
  FAR uint8_t *payload;

  ipv6     = (FAR struct ipv6_hdr_s *)(iob->io_data + iob->io_offset);
  payload  = (FAR uint8_t *)(ipv6 + 1);
  exthdr   = (FAR struct ipv6_extension_s *)payload;
  nxthdr   = ipv6->proto;

  *hdroff  = offsetof(struct ipv6_hdr_s, proto);
  *hdrtype = ipv6->proto;

  /* Traverse up to three extension headers, if the Destination Options
   * Header appears repeatedly, ingore the secondary one and end the search.
   * refer to rfc2460, section-4.1
   */

  while (ipv6_exthdr(nxthdr) && iter++ < 3)
    {
      uint16_t extlen;

      exthdr   = (FAR struct ipv6_extension_s *)payload;
      extlen   = EXTHDR_LEN((unsigned int)exthdr->len);

      switch (nxthdr)
        {
          case NEXT_DESTOPT_EH:
            if (!destopt)
              {
                destopt = true;
              }
            else
              {
                /* This is the secondary Destination Options Header,
                 * end the search
                 */

                goto done;
              }

          case NEXT_ROUTING_EH:
          case NEXT_HOPBYBOT_EH:
              unfraglen = delta + extlen;
              *hdroff = delta;
              *hdrtype = exthdr->nxthdr;
        }

      payload += extlen;
      delta   += extlen;
      nxthdr   = exthdr->nxthdr;
    }

done:
  return unfraglen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_fragin
 *
 * Description:
 *   Handling incoming IPv6 fragment input, the input data
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

int32_t ipv6_fragin(FAR struct net_driver_s *dev)
{
  FAR struct ip_fragsnode_s *node = NULL;
  FAR struct ip_fraglink_s  *fraginfo = NULL;
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

  ipv6_fragin_getinfo(dev->d_iob, fraginfo);

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

      ipv6_fragin_reassemble(node);
      netdev_iob_replace(dev, node->outgoframe);

      /* Free the memory of node */

      kmm_free(node);

      return ipv6_input(dev);
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
 * Name: ipv6_fragout
 *
 * Description:
 *   Execute the ipv6 fragment function. After this work is done, all
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

int32_t ipv6_fragout(FAR struct net_driver_s *dev, uint16_t mtu)
{
  uint16_t unfraglen;
  uint16_t offset = 0;
  uint32_t ipid;
  uint32_t iter;
  uint32_t nfrags;
  uint16_t hdroff;
  uint16_t hdrtype;
  FAR struct iob_s *frag;
  FAR struct ipv6_hdr_s *ref;
  FAR struct ipv6_fragment_extension_s *fraghdr;
  struct iob_queue_s fragq =
    {
      NULL, NULL
    };

  /* Get the length of Unfragmentable Part of the original ipv6 packet,
   * Get the offset and value of nextheader filed in the last extension
   * header of the unfragmentable part.
   */

  unfraglen = ipv6_fragout_getunfraginfo(dev->d_iob, &hdroff, &hdrtype);

  /* Reconstruct I/O Buffer according to MTU, which will reserve
   * the space for the L3 header
   */

  nfrags = ip_fragout_slice(dev->d_iob, PF_INET6, mtu, unfraglen, &fragq);
  assert(nfrags > 1);
  netdev_iob_clear(dev);

  ipid = ++g_ipv6id;

  /* Fill the L3 header into the reserved space */

  for (iter = 0; iter < nfrags; iter++)
    {
      frag = iob_remove_queue(&fragq);

      if (iter == 0)
        {
          ref = (FAR struct ipv6_hdr_s *)(frag->io_data + frag->io_offset);

          /* Update the IPv6 header for the zero fragment */

          ipv6_fragout_buildipv6header(ref, ref, unfraglen,
                frag->io_pktlen - IPv6_HDRLEN, hdroff, NEXT_FRAGMENT_EH);

          /* Build the fragment header for the zero fragment */

          fraghdr = (FAR struct ipv6_fragment_extension_s *)
                    (frag->io_data + frag->io_offset + unfraglen);
          ipv6_fragout_buildipv6fragheader(fraghdr, hdrtype,
                    FRAGHDR_FRAG_MOREFRAGS, ipid);
        }
      else
        {
          uint16_t ipoff = offset - iter * (unfraglen + EXTHDR_FRAG_LEN);

          if (iter < nfrags - 1)
            {
              ipoff |= FRAGHDR_FRAG_MOREFRAGS;
            }

          /* Refer to the zero fragment ipv6 header to construct the ipv6
           * header of non-zero fragment
           */

          ipv6_fragout_buildipv6header(ref,
                  (FAR struct ipv6_hdr_s *)(frag->io_data + frag->io_offset),
                  unfraglen, frag->io_pktlen - IPv6_HDRLEN, hdroff,
                  NEXT_FRAGMENT_EH);

          /* Build extension fragment header for non-zero fragment */

          fraghdr = (FAR struct ipv6_fragment_extension_s *)
                    (frag->io_data + frag->io_offset + unfraglen);
          ipv6_fragout_buildipv6fragheader(fraghdr, hdrtype, ipoff, ipid);
        }

      /* Enqueue this fragment to dev->d_fragout */

      if (iob_tryadd_queue(frag, &dev->d_fragout) < 0)
        {
          goto fail;
        }

      offset += frag->io_pktlen;
    }

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.sent += nfrags - 1;
#endif

  netdev_txnotify_dev(dev);

  return OK;

fail:
  netdev_iob_release(dev);
  iob_free_chain(frag);
  iob_free_queue(&fragq);
  iob_free_queue(&dev->d_fragout);
  --g_ipv6id;
  return -ENOMEM;
}

#endif /* CONFIG_NET_IPv6 && CONFIG_NET_IPFRAG */
