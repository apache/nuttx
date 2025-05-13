/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_ele.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ELE_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/imx9_memorymap.h>
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

/* ELE commands. */

#define ELE_DERIVE_KEY_REQ        0xa9
#define ELE_GET_EVENTS_REQ        0xa2
#define ELE_GET_TRNG_STATE_REQ    0xa4
#define ELE_GET_RNG_REQ           0xcd
#define ELE_FWD_LIFECYCLE_UP_REQ  0x95
#define ELE_OEM_CNTN_AUTH_REQ     0x87
#define ELE_READ_FUSE_REQ         0x97
#define ELE_RELEASE_CONTAINER_REQ 0x89
#define ELE_RELEASE_RDC_REQ       0xc4
#define ELE_START_RNG_REQ         0xa3
#define ELE_VERIFY_IMAGE_REQ      0x88

/* Messaging Unit registers. */

#define ELE_MU_TCR (IMX9_S3MUA_BASE + 0x120)
#define ELE_MU_TSR (IMX9_S3MUA_BASE + 0x124)
#define ELE_MU_RCR (IMX9_S3MUA_BASE + 0x128)
#define ELE_MU_RSR (IMX9_S3MUA_BASE + 0x12c)

#define ELE_RR_NUM        4
#define ELE_TR_NUM        8
#define ELE_MU_TR(i) (IMX9_S3MUA_BASE + 0x200 + (i) * 4)
#define ELE_MU_RR(i) (IMX9_S3MUA_BASE + 0x280 + (i) * 4)

/* Fuse and Status Block lifecycle register. */

#define FSB_LC_REG              0x4751041cUL

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

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ELE_H */
