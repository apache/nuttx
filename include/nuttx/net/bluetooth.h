/****************************************************************************
 * include/nuttx/net/bluetooth.h
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

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#ifdef CONFIG_NET_BLUETOOTH

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
  uint8_t   bm_proto;                         /* Protocol */
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

#endif /* CONFIG_NET_BLUETOOTH */
#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_H */
