/****************************************************************************
 * include/nuttx/coresight/coresight_stm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_STM_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_STM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum stm_trans_type_e
{
  STM_OPTION_GUARANTEED = 0,  /* Blocking, may stall the bus when fifio is full. */
  STM_OPTION_INVARIANT,       /* No-blocking, may lost trace data when fifo is full. */
};

/* enum stp_packet_type - STP packets that an STM driver sends. */

enum stp_packet_type_e
{
  STP_PACKET_DATA = 0,
  STP_PACKET_FLAG,
  STP_PACKET_USER,
  STP_PACKET_MERR,
  STP_PACKET_GERR,
  STP_PACKET_TRIG,
  STP_PACKET_XSYNC,
};

enum stp_packet_flags_e
{
  STP_PACKET_MARKED = 0x1,
  STP_PACKET_TIMESTAMPED  = 0x2,
};

struct coresight_stm_dev_s
{
  struct coresight_dev_s csdev;
  int traceid;                       /* Trace id. */

  /* Used in STM device: start address of extend stimulus port, this memory
   * should be reserved for stm use.
   * Size of this reserved memory equals to (256 x number of ports).
   */

  uintptr_t stimulus_port_addr;

  /* The total number of stimulus port support by this STM. */

  uint32_t numsp;

  /* Maximus bytes this STM can write at a time. */

  uint8_t write_bytes;

  /* Port enable and select register. */

  uint32_t stmsper;
  uint32_t stmspscr;

  /* Hardware Event Tracing. */

  uint32_t stmheer;
  uint32_t stmheter;
  uint32_t stmhebsr;

  /* Bitmap used to indicate whether corresponding stimulus port is
   * guaranteed transactions or invariant timing transactions.
   */

  unsigned long guaranteed[0];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm_set_channel_options
 *
 * Description:
 *   Set a channel's trace mode.
 *
 * Input Parameters:
 *   stmdev  - Pointer to STM device.
 *   channel - Channels to configure.
 *   options - If this channel's trace mode is guaranteed(blocking)
 *             or invariant(noblocking).
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int stm_set_channel_options(FAR struct coresight_stm_dev_s *stmdev,
                            uint32_t channel, uint32_t options);

/****************************************************************************
 * Name: stm_sendpacket
 *
 * Description:
 *   Write data to STM device.
 *
 * Input Parameters:
 *   stmdev  - Pointer to STM device.
 *   type    - Data type.
 *   flag    - Data flags (or attributes).
 *   channel - Channels this data from.
 *   data    - Pointer to the data buffer.
 *   size    - Data size.
 *
 * Returned Value:
 *   Size of data written to STM device; a negative value on failure.
 *
 ****************************************************************************/

ssize_t stm_sendpacket(FAR struct coresight_stm_dev_s *stmdev,
                       enum stp_packet_type_e type,
                       enum stp_packet_flags_e flag, uint32_t channel,
                       FAR const void *data, size_t size);

/****************************************************************************
 * Name: stm_register
 *
 * Description:
 *   Register a STM devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a STM device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_stm_dev_s *
stm_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: stm_unregister
 *
 * Description:
 *   Unregister a STM devices.
 *
 * Input Parameters:
 *   stmdev  - Pointer to the STM device.
 *
 ****************************************************************************/

void stm_unregister(FAR struct coresight_stm_dev_s *stmdev);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_STM_H
