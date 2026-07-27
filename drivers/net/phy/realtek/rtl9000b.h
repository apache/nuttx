/****************************************************************************
 * drivers/net/phy/realtek/rtl9000b.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL9000B_H
#define __DRIVERS_NET_PHY_REALTEK_RTL9000B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/phy.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY ID: IEEE silicon-fixed (Realtek OUI 0x001cc), unaffected by Ax-pin
 * strapping.  Mask 0xfffff000 matches all RTL9000B variants.
 */

#define RTL9000B_PHY_ID           0x001ccb00
#define RTL9000B_PHY_ID_MASK      0xffffff00

/* OP mode commands */

#define OP_CMD_STANDBY            1
#define OP_CMD_NORMAL             2
#define OP_CMD_SLEEP              3

/* Page addresses (reg31 page select) */

#define RTL_PAGE_DEFAULT          0x0a42
#define RTL_PAGE_LINK             0x0a60
#define RTL_PAGE_DVDDIO           0x0a4c
#define RTL_PAGE_PARAM            0x0a01

/* PHY ready polling (page 0xa42 reg 0x10) */

#define RTL_READY_TO              4000
#define RTL_READY_MASK            0x0007
#define RTL_READY_VAL             0x0003

/* Soft reset (standard MCR) */

#define RTL_RST_TO                2000
#define RTL_MCR_RST               0x8000
#define RTL_RST_DONE              0x2100

/* uCode loading poll (indirect 0xb800 bit6) */

#define RTL_UCODE_TO              2000
#define RTL_UCODE_RDY             0x0040

/* Indirect window registers (= reg 27 / reg 28) */

#define RTL_WIN_ADDR              0x1b
#define RTL_WIN_DATA              0x1c

/* OPCR registers (via indirect window) */

#define RTL_OPCR2                 0xdd00
#define RTL_OPCR4                 0xdd20
#define RTL_OPCR_MODE_MSK         0x001f
#define RTL_OPCR4_MSK             0x000f
#define RTL_OPCR_NORMAL           0x0013
#define RTL_OPCR_SLEEP            0x0014
#define RTL_OPCR_STANDBY          0x0012
#define RTL_OPCR4_NORMAL          0x000b

/* General interrupt registers (page 0xa42) */

#define RTL_GINER                 0x12
#define RTL_GINMR                 0x14
#define RTL_GINSR                 0x1d    /* C22 direct, read-clear */

#define RTL_GINTR_FATAL           (1 << 11)
#define RTL_GINTR_LINK            (1 << 4)
#define RTL_GINTR_PHY             (1 << 0)

#define RTL_GINTR_ENABLE \
  (RTL_GINTR_FATAL | RTL_GINTR_LINK | RTL_GINTR_PHY)

/* OP interrupt registers (via indirect window) */

#define RTL_OPINSR1               0xdd08
#define RTL_OPINER1               0xdd0c
#define RTL_OPINMR1               0xdd0e
#define RTL_OPINSR2               0xdd10
#define RTL_OPINER2               0xdd14
#define RTL_OPINMR2               0xdd16
#define RTL_OPINSR3               0xdd18
#define RTL_OPINER3               0xdd1c
#define RTL_OPINMR3               0xdd1e

/* OPINSR1 bit definitions (UV sources) */

#define RTL_UV_AVDD33             (1 << 5)
#define RTL_UV_DVDD               (1 << 4)
#define RTL_UV_AVDD09             (1 << 3)
#define RTL_UV_DVDD09             (1 << 2)
#define RTL_UV_MII                (1 << 1)
#define RTL_UV_MASK               0x00be

/* OPINSR2 bit definitions */

#define RTL_OT_FLAG               (1 << 11)
#define RTL_REC_MASK              0x001f

/* PHYSR2 (Realtek PHY Specific Status Register 2, C22 reg 0x1a).
 * bit2: Real-time link status (1 = link up).  Updates faster than the
 * MSR latch, recommended by the vendor for link detection.
 */

#define RTL_PHYSR2                 0x1a
#define RTL_PHYSR2_LINK            (1 << 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DVDDIO supply selection */

typedef enum
{
  DVDDIO_LDO_1V8    = 1,
  DVDDIO_LDO_2V5    = 2,
  DVDDIO_EXTPWR_1V8 = 4,
  DVDDIO_EXTPWR_2V5 = 5,
  DVDDIO_EXTPWR_3V3 = 6,
} rtl9000b_dvddio_e;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL9000B_H */
