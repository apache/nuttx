/****************************************************************************
 * arch/arm/src/imx9/hardware/imx9_xcache.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Maarten Zanders
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

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_XCACHE_H
#define __ARCH_ARM_SRC_IMX9_IMX9_XCACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx9_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* XCACHE Register Offsets */

#define IMX9_XCACHE_CCR_OFFSET       0x0000  /* Cache Control Register */
#define IMX9_XCACHE_CLCR_OFFSET      0x0004  /* Cache Line Control Register */
#define IMX9_XCACHE_CSAR_OFFSET      0x0008  /* Cache Search Address Register */
#define IMX9_XCACHE_CCVR_OFFSET      0x000C  /* Cache Value Register */

/* Cache Control Register (CCR) */

#define XCACHE_CCR_ENCACHE           (1 << 0)  /* Enable cache */
#define XCACHE_CCR_INVW0             (1 << 24) /* Invalidate Way 0 */
#define XCACHE_CCR_PUSHW0            (1 << 25) /* Push Way 0 */
#define XCACHE_CCR_INVW1             (1 << 26) /* Invalidate Way 1 */
#define XCACHE_CCR_PUSHW1            (1 << 27) /* Push Way 1 */
#define XCACHE_CCR_GO                (1 << 31) /* Initiate command */

/* Cache Line Control Register (CLCR) */

#define XCACHE_CLCR_LGO              (1 << 0)  /* Line command go */
#define XCACHE_CLCR_CACHEADDR_SHIFT  2
#define XCACHE_CLCR_CACHEADDR_MASK   (0x7ff << XCACHE_CLCR_CACHEADDR_SHIFT)
#define XCACHE_CLCR_WSEL             (1 << 14) /* Way select */
#define XCACHE_CLCR_TDSEL            (1 << 16) /* Tag or data select */
#define XCACHE_CLCR_LCIVB            (1 << 20) /* Line command initial valid */
#define XCACHE_CLCR_LCIMB            (1 << 21) /* Line command initial modified */
#define XCACHE_CLCR_LCWAY            (1 << 22) /* Line command way */
#define XCACHE_CLCR_LADSEL           (1 << 26) /* Line Address Select (0: cache, 1: physical) */
#define XCACHE_CLCR_LCMD_SHIFT       24
#define XCACHE_CLCR_LCMD_MASK        (0x3 << XCACHE_CLCR_LCMD_SHIFT) /* Line command */
#define XCACHE_CLCR_LCMD(n)          ((n << XCACHE_CLCR_LCMD_SHIFT) & XCACHE_CLCR_LCMD_MASK)

#define XCACHE_LCMD_SRCH_RW          0b00
#define XCACHE_LCMD_INVALIDATE       0b01
#define XCACHE_LCMD_PUSH             0b10
#define XCACHE_LCMD_CLEAR            0b11

/* Cache Search Address Register */
#define XCACHE_CSAR_PHYADDR_MASK     (0xFFFFFFFD)
#define XCACHE_CSAR_LGO              (1 << 0)

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_XCACHE_H */
