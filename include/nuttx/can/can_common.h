/****************************************************************************
 * include/nuttx/can/can_common.h
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

#ifndef __INCLUDE_NUTTX_CAN_CAN_COMMON_H
#define __INCLUDE_NUTTX_CAN_CAN_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN payload length and DLC definitions according to ISO 11898-1 */

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */

#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Lookup tables convert can_dlc <-> payload len */

EXTERN const uint8_t g_can_dlc_to_len[16];
EXTERN const uint8_t g_len_to_can_dlc[65];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: can_bytes2dlc
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   nbytes - the byte count to convert to a DLC value
 *
 * Returned Value:
 *   The encoded DLC value corresponding to at least that number of bytes.
 *
 ****************************************************************************/

static inline uint8_t can_bytes2dlc(uint8_t nbytes)
{
#if defined(CONFIG_CAN_FD) || defined(CONFIG_NET_CAN_CANFD)
  if (nbytes > CANFD_MAX_DLEN)
    {
      return CANFD_MAX_DLC;
    }
#else
  if (nbytes > CAN_MAX_DLEN)
    {
      return CAN_MAX_DLC;
    }
#endif

  return g_len_to_can_dlc[nbytes];
}

/****************************************************************************
 * Name: can_dlc2bytes
 *
 * Description:
 *   In the CAN FD format, the coding of the DLC differs from the standard
 *   CAN format. The DLC codes 0 to 8 have the same coding as in standard
 *   CAN.  But the codes 9 to 15 all imply a data field of 8 bytes with
 *   standard CAN.  In CAN FD mode, the values 9 to 15 are encoded to values
 *   in the range 12 to 64.
 *
 * Input Parameters:
 *   dlc    - the DLC value to convert to a byte count
 *
 * Returned Value:
 *   The number of bytes corresponding to the DLC value.
 *
 ****************************************************************************/

static inline uint8_t can_dlc2bytes(uint8_t dlc)
{
#if defined(CONFIG_CAN_FD) || defined(CONFIG_NET_CAN_CANFD)
  if (dlc > CANFD_MAX_DLC)
    {
      return CANFD_MAX_DLEN;
    }
#else
  if (dlc > CAN_MAX_DLC)
    {
      return CAN_MAX_DLEN;
    }
#endif

  return g_can_dlc_to_len[dlc];
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_CAN_CAN_COMMON_H */
