/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_tisci_proto.h
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

/* TISCI (Texas Instruments System Control Interface) message protocol
 * definitions for AM62x.  Only the subset used by the NuttX TISCI client is
 * declared here.  Message IDs, flag bits, and on-the-wire structure layouts
 * mirror the canonical definitions in Linux drivers/firmware/ti_sci.h.
 */

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_TISCI_PROTO_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_TISCI_PROTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TISCI message type IDs */

#define TI_SCI_MSG_VERSION            0x0002
#define TI_SCI_MSG_SET_CLOCK_STATE    0x0100
#define TI_SCI_MSG_GET_CLOCK_STATE    0x0101
#define TI_SCI_MSG_SET_CLOCK_FREQ     0x010c
#define TI_SCI_MSG_GET_CLOCK_FREQ     0x010e
#define TI_SCI_MSG_SET_DEVICE_STATE   0x0200
#define TI_SCI_MSG_GET_DEVICE_STATE   0x0201
#define TI_SCI_MSG_SET_DEVICE_RESETS  0x0202
#define TI_SCI_MSG_RM_IRQ_SET         0x1000
#define TI_SCI_MSG_RM_IRQ_RELEASE     0x1001

/* Message header flags */

#define TI_SCI_MSG_FLAG(val)              (1u << (val))
#define TI_SCI_FLAG_REQ_GENERIC_NORESP    0x0
#define TI_SCI_FLAG_REQ_ACK_ON_RECEIVED   TI_SCI_MSG_FLAG(0)
#define TI_SCI_FLAG_REQ_ACK_ON_PROCESSED  TI_SCI_MSG_FLAG(1)
#define TI_SCI_FLAG_RESP_GENERIC_NACK     0x0
#define TI_SCI_FLAG_RESP_GENERIC_ACK      TI_SCI_MSG_FLAG(1)

/* Device-state request: software state values and header flag options */

#define MSG_DEVICE_SW_STATE_AUTO_OFF      0
#define MSG_DEVICE_SW_STATE_RETENTION     1
#define MSG_DEVICE_SW_STATE_ON            2

#define MSG_FLAG_DEVICE_WAKE_ENABLED      TI_SCI_MSG_FLAG(8)
#define MSG_FLAG_DEVICE_RESET_ISO         TI_SCI_MSG_FLAG(9)
#define MSG_FLAG_DEVICE_EXCLUSIVE         TI_SCI_MSG_FLAG(10)

/* Device-state response: hardware state values */

#define MSG_DEVICE_HW_STATE_OFF           0
#define MSG_DEVICE_HW_STATE_ON            1
#define MSG_DEVICE_HW_STATE_TRANS         2

/* Clock-state request: software state values and header flag options */

#define MSG_CLOCK_SW_STATE_UNREQ          0
#define MSG_CLOCK_SW_STATE_AUTO           1
#define MSG_CLOCK_SW_STATE_REQ            2

#define MSG_FLAG_CLOCK_ALLOW_SSC          TI_SCI_MSG_FLAG(8)
#define MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE  TI_SCI_MSG_FLAG(9)
#define MSG_FLAG_CLOCK_INPUT_TERM         TI_SCI_MSG_FLAG(10)

/* When a clk_id does not fit in the 8-bit clk_id field, it is set to 255 and
 * the real id is carried in the 32-bit clk_id_32 field.
 */

#define TI_SCI_CLOCK_ID_INDIRECT          255

/* Resource-management IRQ request: valid_params bit flags */

#define MSG_FLAG_DST_ID_VALID             TI_SCI_MSG_FLAG(0)
#define MSG_FLAG_DST_HOST_IRQ_VALID       TI_SCI_MSG_FLAG(1)
#define MSG_FLAG_IA_ID_VALID              TI_SCI_MSG_FLAG(2)
#define MSG_FLAG_VINT_VALID               TI_SCI_MSG_FLAG(3)
#define MSG_FLAG_GLB_EVNT_VALID           TI_SCI_MSG_FLAG(4)
#define MSG_FLAG_VINT_STS_BIT_VALID       TI_SCI_MSG_FLAG(5)
#define MSG_FLAG_SHOST_VALID              TI_SCI_MSG_FLAG(31)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Common message header (prefixes every request and response) */

begin_packed_struct struct ti_sci_msg_hdr
{
  uint16_t type;
  uint8_t  host;
  uint8_t  seq;
  uint32_t flags;
} end_packed_struct;

/* TI_SCI_MSG_VERSION */

begin_packed_struct struct ti_sci_msg_resp_version
{
  struct ti_sci_msg_hdr hdr;
  char     firmware_description[32];
  uint16_t firmware_revision;
  uint8_t  abi_major;
  uint8_t  abi_minor;
} end_packed_struct;

/* TI_SCI_MSG_SET_DEVICE_STATE / GET_DEVICE_STATE */

begin_packed_struct struct ti_sci_msg_req_set_device_state
{
  struct ti_sci_msg_hdr hdr;
  uint32_t id;
  uint32_t reserved;
  uint8_t  state;
} end_packed_struct;

begin_packed_struct struct ti_sci_msg_req_get_device_state
{
  struct ti_sci_msg_hdr hdr;
  uint32_t id;
} end_packed_struct;

begin_packed_struct struct ti_sci_msg_resp_get_device_state
{
  struct ti_sci_msg_hdr hdr;
  uint32_t context_loss_count;
  uint32_t resets;
  uint8_t  programmed_state;
  uint8_t  current_state;
} end_packed_struct;

/* TI_SCI_MSG_SET_DEVICE_RESETS */

begin_packed_struct struct ti_sci_msg_req_set_device_resets
{
  struct ti_sci_msg_hdr hdr;
  uint32_t id;
  uint32_t resets;
} end_packed_struct;

/* TI_SCI_MSG_SET_CLOCK_STATE */

begin_packed_struct struct ti_sci_msg_req_set_clock_state
{
  struct ti_sci_msg_hdr hdr;
  uint32_t dev_id;
  uint8_t  clk_id;
  uint8_t  request_state;
  uint32_t clk_id_32;
} end_packed_struct;

/* TI_SCI_MSG_SET_CLOCK_FREQ */

begin_packed_struct struct ti_sci_msg_req_set_clock_freq
{
  struct ti_sci_msg_hdr hdr;
  uint32_t dev_id;
  uint64_t min_freq_hz;
  uint64_t target_freq_hz;
  uint64_t max_freq_hz;
  uint8_t  clk_id;
  uint32_t clk_id_32;
} end_packed_struct;

/* TI_SCI_MSG_GET_CLOCK_FREQ */

begin_packed_struct struct ti_sci_msg_req_get_clock_freq
{
  struct ti_sci_msg_hdr hdr;
  uint32_t dev_id;
  uint8_t  clk_id;
  uint32_t clk_id_32;
} end_packed_struct;

begin_packed_struct struct ti_sci_msg_resp_get_clock_freq
{
  struct ti_sci_msg_hdr hdr;
  uint64_t freq_hz;
} end_packed_struct;

/* TI_SCI_MSG_RM_IRQ_SET / RM_IRQ_RELEASE */

begin_packed_struct struct ti_sci_msg_req_manage_irq
{
  struct ti_sci_msg_hdr hdr;
  uint32_t valid_params;
  uint16_t src_id;
  uint16_t src_index;
  uint16_t dst_id;
  uint16_t dst_host_irq;
  uint16_t ia_id;
  uint16_t vint;
  uint16_t global_event;
  uint8_t  vint_status_bit;
  uint8_t  secondary_host;
} end_packed_struct;

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_TISCI_PROTO_H */
