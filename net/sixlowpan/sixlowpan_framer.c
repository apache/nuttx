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
#include <debug.h>

#include "nuttx/net/net.h"
#include "nuttx/net/sixlowpan.h"

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Structure that contains the lengths of the various addressing and
 * security fields in the 802.15.4 header.
 */

struct field_length_s
{
  uint8_t dest_pid_len;    /**<  Length (in bytes) of destination PAN ID field */
  uint8_t dest_addr_len;   /**<  Length (in bytes) of destination address field */
  uint8_t src_pid_len;     /**<  Length (in bytes) of source PAN ID field */
  uint8_t src_addr_len;    /**<  Length (in bytes) of source address field */
  uint8_t aux_sec_len;     /**<  Length (in bytes) of aux security header field */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_addrlen
 *
 * Description:
 *   Return the address length associated with a 2-bit address mode
 *
 * Input parameters:
 *   addrmode - The address mode
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static inline uint8_t sixlowpan_addrlen(uint8_t addrmode)
{
  switch (addrmode)
    {
    case FRAME802154_SHORTADDRMODE:  /* 16-bit address */
      return 2;
    case FRAME802154_LONGADDRMODE:   /* 64-bit address */
      return 8;
    default:
      return 0;
    }
}

/****************************************************************************
 * Function: sixlowpan_addrnull
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
#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
  int i = 2;
#else
  int i = 8;
#endif

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
 * Function: sixlowpan_fieldlengths
 *
 * Description:
 *   Return the lengths associated fields of the IEEE802.15.4 header.
 *
 * Input parameters:
 *   finfo - IEEE802.15.4 header info (input)
 *   flen  - Field length info (output)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_fieldlengths(FAR struct frame802154_s *finfo,
                                   FAR struct field_length_s *flen)
{
  /* Initialize to all zero */

  memset(flen, 0, sizeof(struct field_length_s));

  /* Determine lengths of each field based on fcf and other args */

  if ((finfo->fcf.dest_addr_mode & 3) != 0)
    {
      flen->dest_pid_len = 2;
    }

  if ((finfo->fcf.src_addr_mode & 3) != 0)
    {
      flen->src_pid_len = 2;
    }

  /* Set PAN ID compression bit if src pan id matches dest pan id. */

  if ((finfo->fcf.dest_addr_mode & 3) != 0 &&
      (finfo->fcf.src_addr_mode & 3) != 0 &&
      finfo->src_pid == finfo->dest_pid)
    {
      finfo->fcf.panid_compression = 1;

      /* Compressed header, only do dest pid */
      /* flen->src_pid_len = 0; */
    }

  /* Determine address lengths */

  flen->dest_addr_len = sixlowpan_addrlen(finfo->fcf.dest_addr_mode & 3);
  flen->src_addr_len  = sixlowpan_addrlen(finfo->fcf.src_addr_mode & 3);

  /* Aux security header */

#if 0 /* TODO Aux security header not yet implemented */
  if ((finfo->fcf.security_enabled & 1) != 0)
    {
      switch(finfo->aux_hdr.security_control.key_id_mode)
        {
        case 0:
          flen->aux_sec_len = 5; /* Minimum value */
          break;

        case 1:
          flen->aux_sec_len = 6;
          break;

        case 2:
          flen->aux_sec_len = 10;
          break;

        case 3:
          flen->aux_sec_len = 14;
          break;

        default:
          break;
        }
    }
#endif
}

/****************************************************************************
 * Function: sixlowpan_fieldlengths
 *
 * Description:
 *   Return the lengths associated fields of the IEEE802.15.4 header.
 *
 * Input parameters:
 *   finfo - IEEE802.15.4 header info (input)
 *   flen  - Field length info (output)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sixlowpan_flen_hdrlen(FAR const struct field_length_s *flen)
{
  return 3 + flen->dest_pid_len + flen->dest_addr_len +
         flen->src_pid_len + flen->src_addr_len + flen->aux_sec_len;
}

/****************************************************************************
 * Function: sixlowpan_802154_hdrlen
 *
 * Description:
 *   Calculates the length of the frame header.  This function is meant to
 *   be called by a higher level function, that interfaces to a MAC.
 *
 * Input parameters:
 *   finfo - IEEE802.15.4 header info that specifies the frame to send.
 *
 * Returned Value:
 *   The length of the frame header.
 *
 ****************************************************************************/

static int sixlowpan_802154_hdrlen(FAR struct frame802154_s *finfo)
{
  struct field_length_s flen;

  sixlowpan_fieldlengths(finfo, &flen);
  return sixlowpan_flen_hdrlen(&flen);
}

/****************************************************************************
 * Function: sixlowpan_setup_params
 *
 * Description:
 *   Configure frame parmeters structure.
 *
 * Input parameters:
 *   ieee       - A reference IEEE802.15.4 MAC network device structure.
 *   params     - Where to put the parmeters
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sixlowpan_setup_params(FAR struct ieee802154_driver_s *ieee,
                                   FAR struct frame802154_s *params,
                                   uint16_t dest_panid)
{
  bool rcvrnull;

  /* Initialize all prameters to all zero */

  memset(&params, 0, sizeof(params));

  /* Reset to an empty frame */

  ieee->i_framelen   = 0;
  ieee->i_dataoffset = 0;

  /* Build the FCF (Only non-zero elements need to be initialized). */

  params->fcf.frame_type    = FRAME802154_DATAFRAME;
  params->fcf.frame_pending = ieee->i_pktattrs[PACKETBUF_ATTR_PENDING];

  /* If the output address is NULL in the Rime buf, then it is broadcast
   * on the 802.15.4 network.
   */

  rcvrnull = sixlowpan_addrnull(ieee->i_pktaddrs[PACKETBUF_ADDR_RECEIVER].u8);
  if (rcvrnull)
    {
      params->fcf.ack_required = ieee->i_pktattrs[PACKETBUF_ATTR_MAC_ACK];
    }

  /* Insert IEEE 802.15.4 (2003) version bit. */

  params->fcf.frame_version = FRAME802154_IEEE802154_2003;

  /* Increment and set the data sequence number. */

  if (ieee->i_pktattrs[PACKETBUF_ATTR_MAC_SEQNO] != 0)
    {
      params->seq = ieee->i_pktattrs[PACKETBUF_ATTR_MAC_SEQNO];
    }
  else
    {
      params->seq = ieee->i_dsn++;
      ieee->i_pktattrs[PACKETBUF_ATTR_MAC_SEQNO] = params->seq;
    }

  /* Complete the addressing fields. */
  /* Set the source and destination PAN ID. */

  params->src_pid  = ieee->i_panid;
  params->dest_pid = dest_panid;

  /* If the output address is NULL in the Rime buf, then it is broadcast
   * on the 802.15.4 network.
   */

  if (rcvrnull)
    {
      /* Broadcast requires short address mode. */

      params->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
      params->dest_addr[0] = 0xff;
      params->dest_addr[1] = 0xff;
    }
  else
    {
      /* Copy the destination address */

      rimeaddr_copy((struct rimeaddr_s *)&params->dest_addr,
                    ieee->i_pktaddrs[PACKETBUF_ADDR_RECEIVER].u8);

      /* Use short address mode if so configured */

#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
      params->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
#else
      params->fcf.dest_addr_mode = FRAME802154_LONGADDRMODE;
#endif
    }

  /* Set the source address to the node address assigned to the device */

  rimeaddr_copy((struct rimeaddr_s *)&params->src_addr, &ieee->i_nodeaddr.u8);

  /* Configure the payload address and length */

  params->payload     = FRAME_DATA_START(ieee);
  params->payload_len = FRAME_DATA_SIZE(ieee);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_hdrlen
 *
 * Description:
 *   This function is before the first frame has been sent in order to
 *   determine what the size of the IEEE802.15.4 header will be.  No frame
 *   buffer is required to make this determination.
 *
 * Input parameters:
 *   ieee       - A reference IEEE802.15.4 MAC network device structure.
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_hdrlen(FAR struct ieee802154_driver_s *ieee,
                     uint16_t dest_panid)
{
  struct frame802154_s params;

  /* Set up the frame parameters */

  sixlowpan_setup_params(ieee, &params, dest_panid);

  /* Return the length of the header */

  return sixlowpan_802154_hdrlen(&params);
}

/****************************************************************************
 * Function: sixlowpan_802154_framecreate
 *
 * Description:
 *   Creates a frame for transmission over the air.  This function is meant
 *   to be called by a higher level function, that interfaces to a MAC.
 *
 * Input parameters:
 *   finfo  - Pointer to struct EEE802.15.4 header structure that specifies
 *            the frame to send.
 *   buf    - Pointer to the buffer to use for the frame.
 *   buflen - The length of the buffer to use for the frame.
 *   finfo - I that specifies the frame to send.
 *
 * Returned Value:
 *   The length of the frame header or 0 if there was insufficient space in
 *   the buffer for the frame headers.
 *
 ****************************************************************************/

int sixlowpan_802154_framecreate(FAR struct frame802154_s *finfo,
                                 FAR uint8_t *buf, int buflen)
{
  struct field_length_s flen;
  uint8_t pos;
  int hdrlen;
  int i;

  sixlowpan_fieldlengths(finfo, &flen);

  hdrlen = sixlowpan_flen_hdrlen(&flen);
  if (hdrlen > buflen)
    {
      /* Too little space for headers. */

      return 0;
    }

  /* OK, now we have field lengths.  Time to actually construct
   * the outgoing frame, and store it in the provided buffer
   */

  buf[0] = (finfo->fcf.frame_type         & 7) |
           ((finfo->fcf.security_enabled  & 1) << 3) |
           ((finfo->fcf.frame_pending     & 1) << 4) |
           ((finfo->fcf.ack_required      & 1) << 5) |
           ((finfo->fcf.panid_compression & 1) << 6);
  buf[1] = ((finfo->fcf.dest_addr_mode    & 3) << 2) |
           ((finfo->fcf.frame_version     & 3) << 4) |
           ((finfo->fcf.src_addr_mode     & 3) << 6);

  /* Sequence number */

  buf[2] = finfo->seq;
  pos = 3;

  /* Destination PAN ID */

  if (flen.dest_pid_len == 2)
    {
      buf[pos++] = finfo->dest_pid & 0xff;
      buf[pos++] = (finfo->dest_pid >> 8) & 0xff;
    }

  /* Destination address */

  for (i = flen.dest_addr_len; i > 0; i--)
    {
     buf[pos++] = finfo->dest_addr[i - 1];
    }

  /* Source PAN ID */

  if (flen.src_pid_len == 2)
    {
      buf[pos++] = finfo->src_pid & 0xff;
      buf[pos++] = (finfo->src_pid >> 8) & 0xff;
    }

  /* Source address */

  for (i = flen.src_addr_len; i > 0; i--)
    {
      buf[pos++] = finfo->src_addr[i - 1];
    }

  /* Aux header */

#if 0 /* TODO Aux security header not yet implemented */
  if (flen.aux_sec_len)
    {
      pos += flen.aux_sec_len;
    }
#endif

  DEBUGASSERT(pos == hdrlen);
  return (int)pos;
}

/****************************************************************************
 * Function: sixlowpan_framecreate
 *
 * Description:
 *   This function is called after the IEEE802.15.4 MAC driver polls for
 *   TX data.  It creates the IEEE802.15.4 header in the frame buffer.
 *
 * Input parameters:
 *   ieee       - A reference IEEE802.15.4 MAC network device structure.
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_framecreate(FAR struct ieee802154_driver_s *ieee,
                          uint16_t dest_panid)
{
  struct frame802154_s params;
  int len;
  int ret;

  /* Set up the frame parameters */

  sixlowpan_setup_params(ieee, &params, dest_panid);

  /* Get the length of the header */

  len = sixlowpan_802154_hdrlen(&params);

  /* Allocate space for the header in the frame buffer */

  ret = sixlowpan_frame_hdralloc(ieee, len);
  if (ret < 0)
    {
      wlerr("ERROR: Header too large: %u\n", len);
      return ret;
    }

  /* Then create the frame */

  sixlowpan_802154_framecreate(&params, FRAME_HDR_START(ieee), len);

  wlinfo("Frame type: %02x Data len: %d %u (%u)\n",
         params.fcf.frame_type, len, FRAME_DATA_SIZE(ieee),
         FRAME_SIZE(ieee));
#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
  wlinfo("Dest address: %02x:%02x\n",
         params.dest_addr[0], params.dest_addr[1]);
#else
  wlinfo("Dest address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         params.dest_addr[0], params.dest_addr[1], params.dest_addr[2],
         params.dest_addr[3], params.dest_addr[4], params.dest_addr[5],
         params.dest_addr[6], params.dest_addr[7]);
#endif

  return len;
}

#endif /* CONFIG_NET_6LOWPAN */
