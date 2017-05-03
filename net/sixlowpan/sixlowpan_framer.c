/****************************************************************************
 * net/sixlowpan/sixlowpan_framer.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
 *            Joakim Eriksson <joakime@sics.se>
 *            Joel Hoglund <joel@sics.se>
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

#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nuttx/net/net.h"
#include "nuttx/wireless/ieee802154/ieee802154_mac.h"

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_addrnull
 *
 * Description:
 *   If the output address is NULL in the Rime buf, then it is broadcast
 *   on the 802.15.4 network.
 *
 * Input parameters:
 *   addrmode - The address mode
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static bool sixlowpan_addrnull(FAR uint8_t *addr)
{
  int i = NET_6LOWPAN_RIMEADDR_SIZE;

  while (i-- > 0)
    {
      if (addr[i] != 0x00)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_meta_data
 *
 * Description:
 *   Based on the collected attributes and addresses, construct the MAC meta
 *   data structure that we need to interface with the IEEE802.15.4 MAC.
 *
 * Input Parameters:
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *   meta       - Location to return the corresponding meta data.
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_meta_data(uint16_t dest_panid,
                        FAR struct ieee802154_frame_meta_s *meta)
{
#if 0 /* To be provided */
  /* Set up the frame parameters */

  uint16_t src_panid;
  bool rcvrnull;

  /* Initialize all prameters to all zero */

  memset(meta, 0, sizeof(struct ieee802154_frame_meta_s));

  /* Build the FCF (Only non-zero elements need to be initialized). */

  meta->fcf.frame_type    = FRAME802154_DATAFRAME;
  meta->fcf.frame_pending = g_pktattrs[PACKETBUF_ATTR_PENDING];

  /* If the output address is NULL in the Rime buf, then it is broadcast
   * on the 802.15.4 network.
   */

  rcvrnull = sixlowpan_addrnull(g_pktaddrs[PACKETBUF_ADDR_RECEIVER].u8);
  if (rcvrnull)
    {
      meta->fcf.ack_required = g_pktattrs[PACKETBUF_ATTR_MAC_ACK];
    }

  /* Insert IEEE 802.15.4 (2003) version bit. */

  meta->fcf.frame_version = FRAME802154_IEEE802154_2003;

  /* Complete the addressing fields. */
  /* Get the source PAN ID from the IEEE802.15.4 radio driver */

  src_panid = 0xffff;
  (void)sixlowpan_src_panid(ieee, &src_panid);

  /* Set the source and destination PAN ID. */

  meta->src_pid  = src_panid;
  meta->dest_pid = dest_panid;

  /* If the output address is NULL in the Rime buf, then it is broadcast
   * on the 802.15.4 network.
   */

  if (rcvrnull)
    {
      /* Broadcast requires short address mode. */

      meta->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
      meta->dest_addr[0] = 0xff;
      meta->dest_addr[1] = 0xff;
    }
  else
    {
      /* Copy the destination address */

      rimeaddr_copy((struct rimeaddr_s *)&meta->dest_addr,
                    g_pktaddrs[PACKETBUF_ADDR_RECEIVER].u8);

      /* Use short destination address mode if so configured */

#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
      meta->fcf.dest_addr_mode = FRAME802154_LONGADDRMODE;
#else
      meta->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
#endif
    }

  /* Set the source address to the node address assigned to the device */

  rimeaddr_copy((struct rimeaddr_s *)&meta->src_addr,
                &ieee->i_dev.d_mac.ieee802154);

  /* Use short soruce address mode if so configured */

#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
  meta->fcf.src_addr_mode = FRAME802154_LONGADDRMODE;
#else
  meta->fcf.src_addr_mode = FRAME802154_SHORTADDRMODE;
#endif
#endif

#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: sixlowpan_frame_hdrlen
 *
 * Description:
 *   This function is before the first frame has been sent in order to
 *   determine what the size of the IEEE802.15.4 header will be.  No frame
 *   buffer is required to make this determination.
 *
 * Input parameters:
 *   ieee - A reference IEEE802.15.4 MAC network device structure.
 *   meta - Meta data that describes the MAC header
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_frame_hdrlen(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ieee802154_frame_meta_s *meta)
{
  return ieee->i_get_mhrlen(ieee, meta);
}

/****************************************************************************
 * Name: sixlowpan_frame_submit
 *
 * Description:
 *   This function is called after eiether (1) the IEEE802.15.4 MAC driver
 *   polls for TX data or (2) after the IEEE802.15.4 MAC driver provides a
 *   new incoming frame and the network responds with an outgoing packet.  It
 *   submits any new outgoing frame to the MAC.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   meta  - Meta data that describes the MAC header
 *   frame - The IOB containing the frame to be submitted.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise, a negated errno value is
 *   return on any failure.
 *
 ****************************************************************************/

int sixlowpan_frame_submit(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ieee802154_frame_meta_s *meta,
                           FAR struct iob_s *frame)
{
  return ieee->i_req_data(ieee, meta, frame);
}

#endif /* CONFIG_NET_6LOWPAN */
