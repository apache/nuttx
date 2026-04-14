/****************************************************************************
 * include/netpacket/cangw.h
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

#ifndef __INCLUDE_NETPACKET_CANGW_H
#define __INCLUDE_NETPACKET_CANGW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/can.h>
#include <stdint.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_CANFORWARD

/* Gateway types, the value of rtcanmsg.gwtype */

enum
{
  CGW_TYPE_UNSPEC,
  CGW_TYPE_CAN_CAN,     /* CAN->CAN routing */
  CGW_TYPE_CAN_ETH1722, /* CAN->Ethernet routing */
  CGW_TYPE_ETH1722_CAN, /* Ethernet->CAN routing */
  __CGW_TYPE_MAX
};

#define CGW_TYPE_MAX (__CGW_TYPE_MAX - 1)

struct rtcanmsg
{
  uint8_t  can_family;
  uint8_t  gwtype;
  uint16_t flags;
};

begin_packed_struct struct cgw_frame_mod
{
  struct can_frame cf;
  uint8_t modtype;
}end_packed_struct;

begin_packed_struct struct cgw_fdframe_mod
{
  struct canfd_frame cf;
  uint8_t modtype;
}end_packed_struct;

begin_packed_struct struct cgw_csum_xor
{
  int8_t from_idx;
  int8_t to_idx;
  int8_t result_idx;
  uint8_t init_xor_val;
}end_packed_struct;

begin_packed_struct struct cgw_csum_crc8
{
  int8_t from_idx;
  int8_t to_idx;
  int8_t result_idx;
  uint8_t init_crc_val;
  uint8_t final_xor_val;
  uint8_t crctab[256];
  uint8_t profile;
  uint8_t profile_data[20];
}end_packed_struct;

/* CRC8 profiles (compute CRC for additional data elements - see below) */

enum
{
  CGW_CRC8PRF_UNSPEC,
  CGW_CRC8PRF_1U8,        /* compute one additional u8 value */
  CGW_CRC8PRF_16U8,       /* u8 value table indexed by data[1] & 0xF */
  CGW_CRC8PRF_SFFID_XOR,  /* (can_id & 0xFF) ^ (can_id >> 8 & 0xFF) */
  __CGW_CRC8PRF_MAX
};

/* CAN->CAN rtnetlink attribute definitions */

enum
{
  CGW_UNSPEC,
  CGW_MOD_AND,    /* CAN frame modification binary AND */
  CGW_MOD_OR,     /* CAN frame modification binary OR */
  CGW_MOD_XOR,    /* CAN frame modification binary XOR */
  CGW_MOD_SET,    /* CAN frame modification set alternate values */
  CGW_CS_XOR,     /* set data[] XOR checksum into data[index] */
  CGW_CS_CRC8,    /* set data[] CRC8 checksum into data[index] */
  CGW_HANDLED,    /* number of handled CAN frames */
  CGW_DROPPED,    /* number of dropped CAN frames */
  CGW_SRC_IF,     /* ifindex of source network interface */
  CGW_DST_IF,     /* ifindex of destination network interface */
  CGW_FILTER,     /* specify struct can_filter on source CAN device */
  CGW_DELETED,    /* number of deleted CAN frames (see max_hops param) */
  CGW_LIM_HOPS,   /* limit the number of hops of this specific rule */
  CGW_MOD_UID,    /* user defined identifier for modification updates */
  CGW_FDMOD_AND,  /* CAN FD frame modification binary AND */
  CGW_FDMOD_OR,   /* CAN FD frame modification binary OR */
  CGW_FDMOD_XOR,  /* CAN FD frame modification binary XOR */
  CGW_FDMOD_SET,  /* CAN FD frame modification set alternate values */

  __CGW_MAX
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CGW_MAX (__CGW_MAX - 1)

#define CGW_MOD_FUNCS 4 /* AND OR XOR SET */

/* CAN frame elements that are affected by curr. 3 CAN frame modifications */

#define CGW_MOD_ID      0x01
#define CGW_MOD_DLC     0x02          /* Classical CAN data length code */
#define CGW_MOD_LEN     CGW_MOD_DLC   /* CAN FD (plain) data length */
#define CGW_MOD_DATA    0x04
#define CGW_MOD_FLAGS   0x08          /* CAN FD flags */

#define CGW_FRAME_MODS  4             /* ID DLC/LEN DATA FLAGS */

#define MAX_MODFUNCTIONS (CGW_MOD_FUNCS * CGW_FRAME_MODS)

#define CGW_FLAGS_CAN_ECHO       0x01
#define CGW_FLAGS_CAN_SRC_TSTAMP 0x02
#define CGW_FLAGS_CAN_IIF_TX_OK  0x04
#define CGW_FLAGS_CAN_FD         0x08

/* length of checksum operation parameters.
 * idx = index in CAN frame data[]
 */

#define CGW_CS_XOR_LEN  sizeof(struct cgw_csum_xor)
#define CGW_CS_CRC8_LEN  sizeof(struct cgw_csum_crc8)

#define CGW_MODATTR_LEN sizeof(struct cgw_frame_mod)
#define CGW_FDMODATTR_LEN sizeof(struct cgw_fdframe_mod)

#endif /* CONFIG_NET_CANFORWARD */
#endif /* __INCLUDE_NETPACKET_CANGW_H */
