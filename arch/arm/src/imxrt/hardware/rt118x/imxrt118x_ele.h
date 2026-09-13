/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_ele.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELE_MAX_MSG               255U

#define ELE_CMD_TAG               0x17
#define ELE_VERSION               0x6
#define ELE_VERSION_FW            0x7
#define ELE_RESP_TAG              0xe1
#define ELE_OK                    0xd6

/* ELE commands (subset used by the RT118x port). */

#define ELE_LOAD_FW_REQ           0x02
#define ELE_ENABLE_APC_REQ        0xd2
#define ELE_RELEASE_RDC_REQ       0xc4
#define ELE_GET_FW_VERSION_REQ    0x9d

/* RT118x RDC identifiers used by ELE_RELEASE_RDC_REQ, from NXP
 * MCUXpresso SDK examples/_boards/evkmimxrt1180/board.c.
 */

#define ELE_TRDC_AON_ID           0x74
#define ELE_TRDC_WAKEUP_ID        0x78
#define ELE_TRDC_MEGA_ID          0x82

#define ELE_CORE_CM33_ID          0x1
#define ELE_CORE_CM7_ID           0x2

/* System 3 Messaging Unit A (IMXRT1180RM Ch. 65).
 * The CM33 boots in Secure state and NXP's own SDK uses the Secure alias
 * (0x57540000) rather than the Non-secure alias (0x47540000) for all ELE
 * traffic — writes via the Non-secure alias may not reach the enclave
 * even though reads mirror the same peripheral.
 */

#define ELE_MU_TCR                (IMXRT_S3MUA_BASE + 0x120)
#define ELE_MU_TSR                (IMXRT_S3MUA_BASE + 0x124)
#define ELE_MU_RCR                (IMXRT_S3MUA_BASE + 0x128)
#define ELE_MU_RSR                (IMXRT_S3MUA_BASE + 0x12c)

#define ELE_RR_NUM                4
#define ELE_TR_NUM                8
#define ELE_MU_TR(i)              (IMXRT_S3MUA_BASE + 0x200 + (i) * 4)
#define ELE_MU_RR(i)              (IMXRT_S3MUA_BASE + 0x280 + (i) * 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ele_header_t
{
  union
    {
      uint32_t data;
      struct
        {
          uint32_t version : 8;
          uint32_t size : 8;
          uint32_t command : 8;
          uint32_t tag : 8;
        };
    };
};

struct ele_msg
{
  struct ele_header_t header;
  uint32_t data[(ELE_MAX_MSG - 1)];
};

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H */
