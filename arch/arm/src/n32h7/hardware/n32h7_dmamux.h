/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_dmamux.h
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

 #ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMAMUX_H
 #define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#include "n32h7_memorymap.h"

#if defined(CONFIG_N32H7_N32H76X)
#  include "hardware/n32h76x_dmamux.h"
#else
#  error "Unsupported N32 H7 part"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAMUX1 0
#define DMAMUX2 1

/* Register Offsets *********************************************************/

#define N32_DMAMUX_CXCR_OFFSET(x)    (0x0000U + 0x04U * (x))
#define N32_DMAMUX_STS_OFFSET        0x0080U
#define N32_DMAMUX_CLR_OFFSET        0x0084U
#define N32_DMAMUX_RGXCR_OFFSET(x)   (0x0100U + 0x04U * (x))
#define N32_DMAMUX_RGSTS_OFFSET      0x0180U
#define N32_DMAMUX_RGCLR_OFFSET      0x0184U

/* DMAMUX1 Register Addresses ***********************************************/

#define N32_DMAMUX1_CxCR(x)          (N32_DMAMUX1_BASE + N32_DMAMUX_CXCR_OFFSET(x))
#define N32_DMAMUX1_STS              (N32_DMAMUX1_BASE + N32_DMAMUX_STS_OFFSET)
#define N32_DMAMUX1_CLR              (N32_DMAMUX1_BASE + N32_DMAMUX_CLR_OFFSET)
#define N32_DMAMUX1_RGxCR(x)         (N32_DMAMUX1_BASE + N32_DMAMUX_RGXCR_OFFSET(x))
#define N32_DMAMUX1_RGSTS            (N32_DMAMUX1_BASE + N32_DMAMUX_RGSTS_OFFSET)
#define N32_DMAMUX1_RGCLR            (N32_DMAMUX1_BASE + N32_DMAMUX_RGCLR_OFFSET)

/* DMAMUX2 Register Addresses ***********************************************/

#define N32_DMAMUX2_CxCR(x)          (N32_DMAMUX2_BASE + N32_DMAMUX_CXCR_OFFSET(x))
#define N32_DMAMUX2_STS              (N32_DMAMUX2_BASE + N32_DMAMUX_STS_OFFSET)
#define N32_DMAMUX2_CLR              (N32_DMAMUX2_BASE + N32_DMAMUX_CLR_OFFSET)
#define N32_DMAMUX2_RGxCR(x)         (N32_DMAMUX2_BASE + N32_DMAMUX_RGXCR_OFFSET(x))
#define N32_DMAMUX2_RGSTS            (N32_DMAMUX2_BASE + N32_DMAMUX_RGSTS_OFFSET)
#define N32_DMAMUX2_RGCLR            (N32_DMAMUX2_BASE + N32_DMAMUX_RGCLR_OFFSET)

/* Register Bit Definitions *************************************************/

/* DMAMUX_CXCR bit definitions */
#define DMAMUX_CXCR_REQID_SHIFT      0U
#define DMAMUX_CXCR_REQID_MASK       (0x7FU << DMAMUX_CXCR_REQID_SHIFT)

#define DMAMUX_CXCR_EGE              (1U << 9)  /* Event generation enable */
#define DMAMUX_CXCR_SOIE             (1U << 8)  /* Sync overrun int enable */

#define DMAMUX_CXCR_SYEN             (1U << 16) /* Sync enable */
#define DMAMUX_CXCR_SYPOL_SHIFT      17U
#define DMAMUX_CXCR_SYPOL_MASK       (0x3U << DMAMUX_CXCR_SYPOL_SHIFT)
#define DMAMUX_CXCR_SYPOL_NONE       (0x0U << DMAMUX_CXCR_SYPOL_SHIFT)
#define DMAMUX_CXCR_SYPOL_RISING     (0x1U << DMAMUX_CXCR_SYPOL_SHIFT)
#define DMAMUX_CXCR_SYPOL_FALLING    (0x2U << DMAMUX_CXCR_SYPOL_SHIFT)
#define DMAMUX_CXCR_SYPOL_BOTH       (0x3U << DMAMUX_CXCR_SYPOL_SHIFT)

#define DMAMUX_CXCR_NUMREQ_SHIFT     19U
#define DMAMUX_CXCR_NUMREQ_MASK      (0x1FU << DMAMUX_CXCR_NUMREQ_SHIFT)

#define DMAMUX_CXCR_SYID_SHIFT       24U
#define DMAMUX_CXCR_SYID_MASK        (0xFU << DMAMUX_CXCR_SYID_SHIFT)

/* DMAMUX_STS bit definitions */
#define DMAMUX_STS_SOF(x)            (1U << (x))

/* DMAMUX_CLR bit definitions */
#define DMAMUX_CLR_CSOF(x)           (1U << (x))

/* DMAMUX_RGXCR bit definitions */
#define DMAMUX_RGXCR_SIGID_SHIFT     0U
#define DMAMUX_RGXCR_SIGID_MASK      (0x3FU << DMAMUX_RGXCR_SIGID_SHIFT)

#define DMAMUX_RGXCR_TOVIEN          (1U << 8)  /* Trigger overrun int enable */

#define DMAMUX_RGXCR_GEN             (1U << 16) /* Request generator enable */
#define DMAMUX_RGXCR_GPOL_SHIFT      17U
#define DMAMUX_RGXCR_GPOL_MASK       (0x3U << DMAMUX_RGXCR_GPOL_SHIFT)
#define DMAMUX_RGXCR_GPOL_NONE       (0x0U << DMAMUX_RGXCR_GPOL_SHIFT)
#define DMAMUX_RGXCR_GPOL_RISING     (0x1U << DMAMUX_RGXCR_GPOL_SHIFT)
#define DMAMUX_RGXCR_GPOL_FALLING    (0x2U << DMAMUX_RGXCR_GPOL_SHIFT)
#define DMAMUX_RGXCR_GPOL_BOTH       (0x3U << DMAMUX_RGXCR_GPOL_SHIFT)

#define DMAMUX_RGXCR_GNUMREQ_SHIFT   19U
#define DMAMUX_RGXCR_GNUMREQ_MASK    (0x1FU << DMAMUX_RGXCR_GNUMREQ_SHIFT)

/* DMAMUX_RGSTS bit definitions */
#define DMAMUX_RGSTS_OF(x)           (1U << (x))

/* DMAMUX_RGCLR bit definitions */
#define DMAMUX_RGCLR_COF(x)          (1U << (x))

/****************************************************************************
 * DMA channel mapping
 *
 * XXXXX.DDD.CCCCCCCC
 * C - DMAMUX request
 * D - DMA controller
 * X - free bits
 ****************************************************************************/

#define DMAMAP_MAP(d,c)           ((d) << 8 | (c-1))
#define DMAMAP_CONTROLLER(m)      ((m) >> 8 & 0x07)
#define DMAMAP_REQUEST(m)         ((m) >> 0 & 0xff)

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMAMUX_H */
