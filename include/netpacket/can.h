/****************************************************************************
 * include/netpacket/can.h
 * Definitions for use with AF_PACKET sockets
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

#ifndef __INCLUDE_NETPACKET_CAN_H
#define __INCLUDE_NETPACKET_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Special address description flags for the CAN_ID */

#define CAN_EFF_FLAG 0x80000000  /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000  /* Remote transmission request */
#define CAN_ERR_FLAG 0x20000000  /* Error message frame */

/* Valid bits in CAN ID for frame formats */

#define CAN_SFF_MASK 0x000007ff  /* Standard frame format (SFF) */
#define CAN_EFF_MASK 0x1fffffff  /* Extended frame format (EFF) */
#define CAN_ERR_MASK 0x1fffffff  /* Omit EFF, RTR, ERR flags */

#define CAN_MTU		(sizeof(struct can_frame))
#define CANFD_MTU	(sizeof(struct canfd_frame))

/* PF_CAN protocols */

#define CAN_RAW      1           /* RAW sockets */
#define CAN_BCM      2           /* Broadcast Manager */
#define CAN_TP16     3           /* VAG Transport Protocol v1.6 */
#define CAN_TP20     4           /* VAG Transport Protocol v2.0 */
#define CAN_MCNET    5           /* Bosch MCNet */
#define CAN_ISOTP    6           /* ISO 15765-2 Transport Protocol */
#define CAN_J1939    7           /* SAE J1939 */
#define CAN_NPROTO   8

#define SOL_CAN_BASE 100

#define SOL_CAN_RAW  (SOL_CAN_BASE + CAN_RAW)

/* CAN_RAW socket options */

#define CAN_RAW_FILTER         (__SO_PROTOCOL + 0)
                                 /* set 0 .. n can_filter(s) */
#define CAN_RAW_ERR_FILTER     (__SO_PROTOCOL + 1)
                                 /* set filter for error frames */
#define CAN_RAW_LOOPBACK       (__SO_PROTOCOL + 2)
                                 /* local loopback (default:on) */
#define CAN_RAW_RECV_OWN_MSGS  (__SO_PROTOCOL + 3)
                                 /* receive my own msgs (default:off) */
#define CAN_RAW_FD_FRAMES      (__SO_PROTOCOL + 4)
                                 /* allow CAN FD frames (default:off) */
#define CAN_RAW_JOIN_FILTERS   (__SO_PROTOCOL + 5)
                                 /* all filters must match to trigger */
#define CAN_RAW_TX_DEADLINE    (__SO_PROTOCOL + 6)
                                 /* Abort frame when deadline passed */

/* CAN filter support (Hardware level filtering) ****************************/

/* Some CAN hardware supports a notion of prioritizing messages that match
 * filters. Only two priority levels are currently supported and are encoded
 * as defined below:
 */

#define CAN_MSGPRIO_LOW   0
#define CAN_MSGPRIO_HIGH  1

/* Filter type.  Not all CAN hardware will support all filter types. */

#define CAN_FILTER_MASK   0  /* Address match under a mask */
#define CAN_FILTER_DUAL   1  /* Dual address match */
#define CAN_FILTER_RANGE  2  /* Match a range of addresses */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Controller Area Network Identifier structure
 *
 *   Bit 0-28: CAN identifier (11/29 bit)
 *   Bit 29:   Error message frame flag (0 = data frame, 1 = error message)
 *   Bit 30:   Remote transmission request flag (1 = rtr frame)
 *   Bit 31:   Frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */

typedef uint32_t canid_t;

/* The sockaddr structure for CAN sockets
 *
 *   can_family:  Address family number AF_CAN.
 *   can_ifindex: CAN network interface index.
 *   can_addr:    Protocol specific address information
 */

struct sockaddr_can
{
  sa_family_t can_family;
  int16_t     can_ifindex;
  union
  {
    /* Transport protocol class address information */

    struct
    {
      canid_t rx_id;
      canid_t tx_id;
    } tp;

    /* J1939 address information */

    struct
    {
      /* 8 byte name when using dynamic addressing */

      uint64_t name;

      /* pgn:
       *   8 bit: PS in PDU2 case, else 0
       *   8 bit: PF
       *   1 bit: DP
       *   1 bit: reserved
       */

      uint32_t pgn;

      /* 1 byte address */

      uint8_t addr;
    } j1939;
  } can_addr;
};

#endif /* __INCLUDE_NETPACKET_CAN_H */
