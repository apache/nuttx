/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_trdc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/imx9_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_TRDC_HWCFG0                (IMX9_TRDC_BASE + 0xf0)

#define IMX9_MBC0_MEM_GLBAC(n)          (0x20 + (n << 2))
#define IMX9_MBC_MEM_BLK_CFG_0(m, n)    (0x200 * m + 0x40 + (n << 2))
#define IMX9_MBC_MEM_BLK_CFG_I(m, n, i) (0x200 * m + 0x40 + (80 << 2) + (i - 1) * 0x28 + (n << 2))
#define IMX9_MRC0_DOM_RGD_W(m, n)       (0x100 * m + 0x40 + (n << 3))

#define ELE_MAX_MSG             255U
#define AHAB_VERSION            0x6
#define AHAB_CMD_TAG            0x17
#define AHAB_RESP_TAG           0xe1
#define ELE_RELEASE_RDC_REQ     0xc4
#define ELE_READ_FUSE_REQ       0x97
#define ELE_OK                  0xd6

#define FSB_BASE                0x47510000UL
#define FSB_SHADOW_OFF          0x8000UL

#define BLK_CTRL_NS_ANOMIX_BASE  IMX9_BLK_CTRL_NS_AONMIX1_BASE

#define ELE_MU_TCR (IMX9_S3MUA_BASE+ 0x120)
#define ELE_MU_TSR (IMX9_S3MUA_BASE+ 0x124)
#define ELE_MU_RCR (IMX9_S3MUA_BASE+ 0x128)
#define ELE_MU_RSR (IMX9_S3MUA_BASE+ 0x12c)

#define ELE_RR_NUM        4
#define ELE_TR_NUM        8
#define ELE_MU_TR(i) (IMX9_S3MUA_BASE + 0x200 + (i) * 4)
#define ELE_MU_RR(i) (IMX9_S3MUA_BASE + 0x280 + (i) * 4)

#define DID_NUM 16
#define MBC_MAX_NUM 4
#define MRC_MAX_NUM 2
#define MBC_NUM(HWCFG) ((HWCFG >> 16) & 0xF)
#define MRC_NUM(HWCFG) ((HWCFG >> 24) & 0x1F)

#define MBC_BLK_NUM(GLBCFG) (GLBCFG & 0x3FF)
#define MRC_RGN_NUM(GLBCFG) (GLBCFG & 0x1F)

#define GLBAC_SETTING_MASK (0x7777)
#define GLBAC_LOCK_MASK	BIT(31)

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

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H */
