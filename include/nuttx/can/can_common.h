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
  if (nbytes <= 8)
    {
      return nbytes;
    }
#if defined(CONFIG_CAN_FD) || defined(CONFIG_NET_CAN_CANFD)
  else if (nbytes <= 12)
    {
      return 9;
    }
  else if (nbytes <= 16)
    {
      return 10;
    }
  else if (nbytes <= 20)
    {
      return 11;
    }
  else if (nbytes <= 24)
    {
      return 12;
    }
  else if (nbytes <= 32)
    {
      return 13;
    }
  else if (nbytes <= 48)
    {
      return 14;
    }
  else /* if (nbytes <= 64) */
    {
      return 15;
    }
#else
  else
    {
      return 8;
    }
#endif
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
  if (dlc > 8)
    {
#if defined(CONFIG_CAN_FD) || defined(CONFIG_NET_CAN_CANFD)
      switch (dlc)
        {
          case 9:
            return 12;

          case 10:
            return 16;

          case 11:
            return 20;

          case 12:
            return 24;

          case 13:
            return 32;

          case 14:
            return 48;

          default:
          case 15:
            return 64;
        }
#else
      return 8;
#endif
    }

  return dlc;
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_CAN_CAN_COMMON_H */
