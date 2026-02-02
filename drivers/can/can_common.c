/****************************************************************************
 * drivers/can/can_common.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/can.h>
#include <nuttx/can/can_common.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* DLC to payload length conversion table for CAN and CAN FD
 *
 * According to ISO 11898-1 (Classic CAN) and ISO 11898-7 (CAN FD):
 *
 * DLC 0-8:  Direct mapping (0-8 bytes) - Valid for both CAN and CAN FD
 * DLC 9:    12 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 10:   16 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 11:   20 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 12:   24 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 13:   32 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 14:   48 bytes - CAN FD only (8 bytes in Classic CAN)
 * DLC 15:   64 bytes - CAN FD only (8 bytes in Classic CAN)
 *
 * Note: In Classic CAN, DLC values > 8 are treated as 8 by hardware,
 * but this table provides the CAN FD mapping for software use.
 */

const uint8_t g_can_dlc_to_len[16] =
{
  0, 1, 2, 3, 4, 5, 6, 7, 8,    /* DLC 0-8: Direct mapping */
  12, 16, 20, 24, 32, 48, 64    /* DLC 9-15: CAN FD extended lengths */
};

/* Payload length to DLC conversion table
 *
 * Maps byte count (0-64) to minimum required DLC value.
 * For byte counts between standard DLC values, rounds up to next DLC.
 *
 * Examples:
 *   - 0 bytes  -> DLC 0
 *   - 1-8 bytes -> DLC 1-8 (direct mapping)
 *   - 9-12 bytes -> DLC 9 (allocates 12 bytes in CAN FD)
 *   - 13-16 bytes -> DLC 10 (allocates 16 bytes in CAN FD)
 *   - 17-20 bytes -> DLC 11 (allocates 20 bytes in CAN FD)
 *   - 21-24 bytes -> DLC 12 (allocates 24 bytes in CAN FD)
 *   - 25-32 bytes -> DLC 13 (allocates 32 bytes in CAN FD)
 *   - 33-48 bytes -> DLC 14 (allocates 48 bytes in CAN FD)
 *   - 49-64 bytes -> DLC 15 (allocates 64 bytes in CAN FD)
 */

const uint8_t g_len_to_can_dlc[65] =
{
  0,                                  /* 0 bytes -> DLC 0 */
  1, 2, 3, 4, 5, 6, 7, 8,             /* 1-8 bytes -> DLC 1-8 */
  9, 9, 9, 9,                         /* 9-12 bytes -> DLC 9 */
  10, 10, 10, 10,                     /* 13-16 bytes -> DLC 10 */
  11, 11, 11, 11,                     /* 17-20 bytes -> DLC 11 */
  12, 12, 12, 12,                     /* 21-24 bytes -> DLC 12 */
  13, 13, 13, 13, 13, 13, 13, 13,     /* 25-32 bytes -> DLC 13 */
  14, 14, 14, 14, 14, 14, 14, 14,     /* 33-40 bytes -> DLC 14 */
  14, 14, 14, 14, 14, 14, 14, 14,     /* 41-48 bytes -> DLC 14 */
  15, 15, 15, 15, 15, 15, 15, 15,     /* 49-56 bytes -> DLC 15 */
  15, 15, 15, 15, 15, 15, 15, 15      /* 57-64 bytes -> DLC 15 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
