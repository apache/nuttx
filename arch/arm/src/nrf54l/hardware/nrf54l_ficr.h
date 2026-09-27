/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_ficr.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_FICR_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_FICR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_FICR_INFO_CONFIGID_OFFSET    0x0300                 /* Device configuration identifier */
#define NRF54L_FICR_INFO_DEVICEID_OFFSET(n) (0x0304 + ((n) << 2))  /* Device identifier word n */
#define NRF54L_FICR_INFO_UUID_OFFSET(n)     (0x030c + ((n) << 2))  /* UUID word n */
#define NRF54L_FICR_INFO_PART_OFFSET        0x031c                 /* Part number */
#define NRF54L_FICR_INFO_VARIANT_OFFSET     0x0320                 /* Part variant */
#define NRF54L_FICR_INFO_PACKAGE_OFFSET     0x0324                 /* Package variant */
#define NRF54L_FICR_INFO_RAM_OFFSET         0x0328                 /* RAM size in KiB */
#define NRF54L_FICR_INFO_RRAM_OFFSET        0x032c                 /* RRAM size in KiB */
#define NRF54L_FICR_ER_OFFSET(n)            (0x0380 + ((n) << 2))  /* Encryption root word n */
#define NRF54L_FICR_IR_OFFSET(n)            (0x0390 + ((n) << 2))  /* Identity root word n */
#define NRF54L_FICR_DEVICEADDRTYPE_OFFSET   0x03a0                 /* Device address type */
#define NRF54L_FICR_DEVICEADDR_OFFSET(n)    (0x03a4 + ((n) << 2))  /* Device address word n */
#define NRF54L_FICR_TRIM_ADDR_OFFSET(n)     (0x0400 + ((n) << 3))  /* Factory trim destination n */
#define NRF54L_FICR_TRIM_DATA_OFFSET(n)     (0x0404 + ((n) << 3))  /* Factory trim value n */
#define NRF54L_FICR_NFC_TAGHEADER_OFFSET(n) (0x0600 + ((n) << 2))  /* NFC tag header word n */
#define NRF54L_FICR_XOSC32MTRIM_OFFSET      0x0620                 /* HFXO capacitance trim */
#define NRF54L_FICR_XOSC32KTRIM_OFFSET      0x0624                 /* LFXO capacitance trim */

/* Silicon identification used by the startup errata */

#define NRF54L_FICR_PART_OFFSET         0x0340 /* Silicon part identifier */
#define NRF54L_FICR_VARIANT_OFFSET      0x0344 /* Silicon revision identifier */

/* Register addresses *******************************************************/

#define NRF54L_FICR_INFO_CONFIGID    (NRF54L_FICR_BASE + NRF54L_FICR_INFO_CONFIGID_OFFSET)
#define NRF54L_FICR_INFO_DEVICEID(n) (NRF54L_FICR_BASE + NRF54L_FICR_INFO_DEVICEID_OFFSET(n))
#define NRF54L_FICR_INFO_UUID(n)     (NRF54L_FICR_BASE + NRF54L_FICR_INFO_UUID_OFFSET(n))
#define NRF54L_FICR_INFO_PART        (NRF54L_FICR_BASE + NRF54L_FICR_INFO_PART_OFFSET)
#define NRF54L_FICR_INFO_VARIANT     (NRF54L_FICR_BASE + NRF54L_FICR_INFO_VARIANT_OFFSET)
#define NRF54L_FICR_INFO_PACKAGE     (NRF54L_FICR_BASE + NRF54L_FICR_INFO_PACKAGE_OFFSET)
#define NRF54L_FICR_INFO_RAM         (NRF54L_FICR_BASE + NRF54L_FICR_INFO_RAM_OFFSET)
#define NRF54L_FICR_INFO_RRAM        (NRF54L_FICR_BASE + NRF54L_FICR_INFO_RRAM_OFFSET)
#define NRF54L_FICR_PART             (NRF54L_FICR_BASE + NRF54L_FICR_PART_OFFSET)
#define NRF54L_FICR_VARIANT          (NRF54L_FICR_BASE + NRF54L_FICR_VARIANT_OFFSET)
#define NRF54L_FICR_ER(n)            (NRF54L_FICR_BASE + NRF54L_FICR_ER_OFFSET(n))
#define NRF54L_FICR_IR(n)            (NRF54L_FICR_BASE + NRF54L_FICR_IR_OFFSET(n))
#define NRF54L_FICR_DEVICEADDRTYPE   (NRF54L_FICR_BASE + NRF54L_FICR_DEVICEADDRTYPE_OFFSET)
#define NRF54L_FICR_DEVICEADDR(n)    (NRF54L_FICR_BASE + NRF54L_FICR_DEVICEADDR_OFFSET(n))
#define NRF54L_FICR_TRIM_ADDR(n)     (NRF54L_FICR_BASE + NRF54L_FICR_TRIM_ADDR_OFFSET(n))
#define NRF54L_FICR_TRIM_DATA(n)     (NRF54L_FICR_BASE + NRF54L_FICR_TRIM_DATA_OFFSET(n))
#define NRF54L_FICR_NFC_TAGHEADER(n) (NRF54L_FICR_BASE + NRF54L_FICR_NFC_TAGHEADER_OFFSET(n))
#define NRF54L_FICR_XOSC32MTRIM      (NRF54L_FICR_BASE + NRF54L_FICR_XOSC32MTRIM_OFFSET)
#define NRF54L_FICR_XOSC32KTRIM      (NRF54L_FICR_BASE + NRF54L_FICR_XOSC32KTRIM_OFFSET)

/* Register bit definitions *************************************************/

#define FICR_DEVICEADDRTYPE_RANDOM (1 << 0)

#define FICR_XOSC32MTRIM_SLOPE_SHIFT  (0)
#define FICR_XOSC32MTRIM_SLOPE_MASK   (0x1ff << FICR_XOSC32MTRIM_SLOPE_SHIFT)
#define FICR_XOSC32MTRIM_OFFSET_SHIFT (16)
#define FICR_XOSC32MTRIM_OFFSET_MASK  (0x3ff << FICR_XOSC32MTRIM_OFFSET_SHIFT)

#define FICR_XOSC32KTRIM_SLOPE_SHIFT  (0)
#define FICR_XOSC32KTRIM_SLOPE_MASK   (0x1ff << FICR_XOSC32KTRIM_SLOPE_SHIFT)
#define FICR_XOSC32KTRIM_OFFSET_SHIFT (16)
#define FICR_XOSC32KTRIM_OFFSET_MASK  (0x3ff << FICR_XOSC32KTRIM_OFFSET_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_FICR_H */
