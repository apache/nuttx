/****************************************************************************
 * include/nuttx/net/bluetooth.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************/

#ifndef  __INCLUDE_NUTTX_NET_BLUETOOTH_H
#define  __INCLUDE_NUTTX_NET_BLUETOOTH_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_hci.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLUETOOTH_MAX_FRAMELEN
 * Maximum amount of data that can fit in a buffer.
 *
 * The biggest foreseeable buffer size requirement right now comes from
 * the Bluetooth 4.2 SMP MTU which is 65. This then become 65 + 4 (L2CAP
 * header) + 4 (ACL header) + 1 (H4 header) = 74. This also covers the
 * biggest HCI commands and events which are a bit under the 70 byte
 * mark.
 */

#define BLUETOOTH_L2CAP_HDRLEN  4  /* Size of L2CAP header */
#define BLUETOOTH_ACL_HDRLEN    4  /* Size of ACL header */
#define BLUETOOTH_H4_HDRLEN     1  /* Size of H4 header */

#define BLUETOOTH_MAX_HDRLEN \
  (BLUETOOTH_L2CAP_HDRLEN + BLUETOOTH_ACL_HDRLEN + BLUETOOTH_H4_HDRLEN)

#define BLUETOOTH_SMP_MTU       65
#define BLUETOOTH_MAX_MTU       70

#define BLUETOOTH_MAX_FRAMELEN  (BLUETOOTH_MAX_MTU + BLUETOOTH_MAX_HDRLEN)

#define BLUETOOTH_ADDRSIZE      6
#define BLUETOOTH_ADDRCOPY(d,s) memcpy((d),(s),BLUETOOTH_ADDRSIZE)
#define BLUETOOTH_ADDRCMP(a,b)  (memcmp((a),(b),BLUETOOTH_ADDRSIZE) == 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the form of the meta-data that accompanies frames received from
 * the Bluetooth stack.
 */

struct bluetooth_frame_meta_s
{
  bt_addr_t bm_raddr;                         /* Connected remote address */
  uint8_t bm_channel;                         /* Connection channel */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_input
 *
 * Description:
 *   Handle incoming Bluetooth input
 *
 *   This function is called when the radio device driver has received an
 *   frame from the network.  The frame from the device driver must be
 *   provided in by the IOB frame argument of the  function call:
 *
 *   - The frame data is in the IOB io_data[] buffer,
 *   - The length of the frame is in the IOB io_len field, and
 *   - The offset past and radio MAC header is provided in the io_offset
 *     field.
 *
 *   The frame argument may refer to a single frame (a list of length one)
 *   or may it be the head of a list of multiple frames.
 *
 *   - The io_flink field points to the next frame in the list (if enable)
 *   - The last frame in the list will have io_flink == NULL.
 *
 * Input Parameters:
 *   radio       The radio network driver interface.
 *   framelist - The head of an incoming list of frames.  Normally this
 *               would be a single frame.  A list may be provided if
 *               appropriate, however.
 *   meta      - Meta data characterizing the received frame.
 *
 *               If there are multiple frames in the list, this metadata
 *               must apply to all of the frames in the list.
 *
 * Returned Value:
 *   OK    The Bluetooth has been processed  and can be deleted
 *   ERROR Hold the Bluetooth and try again later. There is a listening
 *         socket but no recv in place to catch the Bluetooth yet.
 *         Useful when a packet arrives before a recv call is in place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

struct radio_driver_s;        /* Forward reference */
struct bluetooth_data_ind_s;  /* Forward reference */
struct iob_s;                 /* Forward reference */

int bluetooth_input(FAR struct radio_driver_s *radio,
                     FAR struct iob_s *framelist,
                     FAR struct bluetooth_frame_meta_s *meta);

#endif /*  __INCLUDE_NUTTX_NET_BLUETOOTH_H */
