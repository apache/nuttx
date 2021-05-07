/****************************************************************************
 * include/nuttx/net/ieee802154.h
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

#ifndef __INCLUDE_NUTTX_NET_IEEE802154_H
#define __INCLUDE_NUTTX_NET_IEEE802154_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_input
 *
 * Description:
 *   Handle incoming IEEE 802.15.4 input
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
 *   OK    The IEEE 802.15.4 has been processed  and can be deleted
 *   ERROR Hold the IEEE 802.15.4 and try again later. There is a listening
 *         socket but no recv in place to catch the IEEE 802.15.4 yet.
 *         Useful when a packet arrives before a recv call is in place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

struct radio_driver_s;        /* Forward reference */
struct ieee802154_data_ind_s; /* Forward reference */
struct iob_s;                 /* Forward reference */

int ieee802154_input(FAR struct radio_driver_s *radio,
                     FAR struct iob_s *framelist,
                     FAR struct ieee802154_data_ind_s *meta);

#endif /* __INCLUDE_NUTTX_NET_IEEE802154_H */
