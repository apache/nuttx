/****************************************************************************
 * include/nuttx/net/radiodev.h
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
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

#ifndef __INCLUDE_NUTTX_NET_RADIODEV_H
#define __INCLUDE_NUTTX_NET_RADIODEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_BLUETOOTH) || \
    defined (CONFIG_NET_IEEE802154)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Different packet radios may have different properties.  If there are
 * multiple packet radios, then those properties have to be queried at
 * run time.  This information is provided to the 6LoWPAN network via the
 * following structure.
 */

struct radiodev_properties_s
{
  uint8_t sp_addrlen;                 /* Length of an address */
  uint8_t sp_framelen;                /* Fixed packet/frame size (up to 255) */
  struct netdev_varaddr_s sp_mcast;   /* Multicast address */
  struct netdev_varaddr_s sp_bcast;   /* Broadcast address */
#ifdef CONFIG_NET_STARPOINT
  struct netdev_varaddr_s sp_hubnode; /* Address of the hub node in a star */
#endif
};

/* The device structure for radio network device differs from the standard
 * Ethernet MAC device structure.  The main reason for this difference is
 * that fragmentation must be supported.
 *
 * The radio network driver does not use the d_buf packet buffer directly.
 * Rather, it uses a list smaller frame buffers.
 *
 *   - Outgoing frame data is provided in an IOB in the via the
 *     r_req_data() interface method each time that the radio needs to
 *     send more data.  The length of the frame is provided in the io_len
 *     field of the IOB.
 *
 *     Outgoing frames are generated when the radio network driver calls
 *     the devif_poll(), devif_timer(), sixlowpan_input(), or
 *     ieee802154_input() interfaces.  In each case, the radio driver must
 *     provide a working buffer in the d_buf pointer.  A special form of
 *     the packet buffer must be used, struct sixlowpan_reassbuf_s.  This
 *     special for includes appended data for managing reassembly of packets.
 *
 *   - Received frames are provided by radio network driver to the network
 *     via an IOB parameter in the sixlowpan_input() pr ieee802154_input()
 *     interface.  The length of the frame is io_len.
 *
 *     Again, the radio network driver must provide an instance of struct
 *     sixlowpan_reassbuf_s as the packet buffer in the d_buf field.  This
 *     driver-provided data will only be used if the the receive frames are
 *     not fragmented.
 *
 *   - Received 6LoWPAN frames and will be uncompressed and possibly
 *     reassembled in resassembled the d_buf;  d_len will hold the size of
 *     the reassembled packet.
 *
 *     For fagemented frames, d_buf provided by radio driver will not be
 *     used.  6LoWPAN must handle multiple reassemblies from different
 *     sources simultaneously.  To support this, 6LoWPAN will allocate a
 *     unique reassembly buffer for each active reassembly, based on the
 *     reassembly tag and source radio address.  These reassembly buffers
 *     are managed entirely by the 6LoWPAN layer.
 *
 * This is accomplished by "inheriting" the standard 'struct net_driver_s'
 * and appending the frame buffer as well as other metadata needed to
 * manage the fragmentation.  'struct radio_driver_s' is cast
 * compatible with 'struct net_driver_s' when dev->d_lltype ==
 * NET_LL_IEEE802154 or dev->d_lltype == NET_LL_PKTRADIO.
 *
 * The radio network driver has reponsibility for initializing this
 * structure.  In general, all fields must be set to NULL.  In addition:
 *
 * 1. On a TX poll, the radio network driver should provide its driver
 *    structure along is (single) reassemby buffer provided at d_buf.
 *    During the course of the poll, the networking layer may generate
 *    outgoing frames.  These frames will by provided to the radio network
 *    driver via the req_data() method.
 *
 *    After sending each frame through the radio, the MAC driver must
 *    return the frame to the pool of free IOBs using the iob_free().
 *
 * 2. When receiving data both buffers must be provided for 6LoWPAN
 *    frames;  PF_IEEE802154 frames do not require the d_buf.
 *
 *    The radio driver should receive the frame data directly into the
 *    payload area of an IOB frame structure.  That IOB structure may be
 *    obtained using the iob_alloc() function.
 *
 *    For 6LoWPAN, fragmented packets will be reassembled using allocated
 *    reassembly buffers that are managed by the 6LoWPAN layer.  The radio
 *    driver must still provide its (single) reassembly buffer in d_buf;
 *    that buffer is still used for the case where the packet is not
 *    fragmented into many frames.  In either case, the packet buffer will
 *    have a size of advertised MTU of the protocol, CONFIG_NET_6LOWPAN_PKTSIZE,
 *    plus CONFIG_NET_GUARDSIZE and some additional overhead for reassembly
 *    state data.
 *
 *    The radio network driver should then inform the network of the recipt
 *    of a frame by calling sixlowpan_input() or ieee802154_input().  That
 *    single frame (or, perhaps, list of frames) should be provided as
 *    second argument of that call.
 *
 *    The network will free the IOB by calling iob_free after it has
 *    processed the incoming frame.  As a complexity, the result of
 *    receiving a frame may be that the network may respond provide an
 *    outgoing frames in the via a nested call to the req_data() method.
 */

struct iob_s;  /* Forward reference */

struct radio_driver_s
{
  /* This definition must appear first in the structure definition to
   * assure cast compatibility.
   */

  struct net_driver_s r_dev;

  /* Radio network driver-specific definitions follow. */

#ifdef CONFIG_WIRELESS_IEEE802154
  /* The msdu_handle is basically an id for the frame.  The standard just
   * says that the next highest layer should determine it.  It is used in
   * three places
   *
   * 1. When you do that data request
   * 2. When the transmission is complete, the conf_data is called with
   *    that handle so that the user can be notified of the frames success/
   *    failure
   * 3. For a req_purge, to basically "cancel" the transaction.  This is
   *    often particularly useful on a coordinator that has indirect data
   *    waiting to be requested from another device
   *
   * Here is a simple frame counter.
   */

  uint8_t r_msdu_handle;
#endif

  /* MAC network driver callback functions **********************************/
  /**************************************************************************
   * Name: r_get_mhrlen
   *
   * Description:
   *   Calculate the MAC header length given the frame meta-data.
   *
   * Input Parameters:
   *   netdev    - The networkd device that will mediate the MAC interface
   *   meta      - Obfuscated metadata structure needed to recreate the
   *               radio MAC header
   *
   * Returned Value:
   *   A non-negative MAC headeer length is returned on success; a negated
   *   errno value is returned on any failure.
   *
   **************************************************************************/

  CODE int (*r_get_mhrlen)(FAR struct radio_driver_s *netdev,
                           FAR const void *meta);

  /**************************************************************************
   * Name: r_req_data
   *
   * Description:
   *   Requests the transfer of a list of frames to the MAC.
   *
   * Input Parameters:
   *   netdev    - The network device that will mediate the MAC interface
   *   meta      - Obfuscated metadata structure needed to create the radio
   *               MAC header
   *   framelist - Head of a list of frames to be transferred.
   *
   * Returned Value:
   *   Zero (OK) returned on success; a negated errno value is returned on
   *   any failure.
   *
   **************************************************************************/

  CODE int (*r_req_data)(FAR struct radio_driver_s *netdev,
                         FAR const void *meta, FAR struct iob_s *framelist);

  /**************************************************************************
   * Name: r_properties
   *
   * Description:
   *   Different packet radios may have different properties.  If there are
   *   multiple packet radios, then those properties have to be queried at
   *   run time.  This information is provided to the 6LoWPAN network via the
   *   following structure.
   *
   * Input Parameters:
   *   netdev     - The network device to be queried
   *   properties - Location where radio properities will be returned.
   *
   * Returned Value:
   *   Zero (OK) returned on success; a negated errno value is returned on
   *   any failure.
   *
   **************************************************************************/

  CODE int (*r_properties)(FAR struct radio_driver_s *netdev,
                           FAR struct radiodev_properties_s *properties);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET_6LOWPAN || CONFIG_NET_BLUETOOTH || CONFIG_NET_IEEE802154 */
#endif /* __INCLUDE_NUTTX_NET_RADIODEV_H */
