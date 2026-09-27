/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_cordic.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_CORDIC_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_CORDIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_CORDIC_CTRLSTS_OFFSET   0x0000   /* Control/status register */
#define N32_CORDIC_WDATA_OFFSET     0x0004   /* Write data register */
#define N32_CORDIC_RDATA_OFFSET     0x0008   /* Read data register */

/* Register Addresses (assuming N32_CORDIC_BASE defined in chip.h) **********/

#define N32_CORDIC_CTRLSTS          (N32_CORDIC_BASE + N32_CORDIC_CTRLSTS_OFFSET)
#define N32_CORDIC_WDATA            (N32_CORDIC_BASE + N32_CORDIC_WDATA_OFFSET)
#define N32_CORDIC_RDATA            (N32_CORDIC_BASE + N32_CORDIC_RDATA_OFFSET)

/* CTRLSTS Bitfield Definitions *********************************************/

#define CORDIC_CTRLSTS_FUNC_SHIFT      0
#define CORDIC_CTRLSTS_FUNC_MASK       (0xf << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_COS      (0 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_SIN      (1 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_PHASE    (2 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_MODULUS  (3 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_ARCTAN   (4 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_HB_COS   (5 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_HB_SIN   (6 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_HB_ARCT  (7 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_NATL     (8 << CORDIC_CTRLSTS_FUNC_SHIFT)
#  define CORDIC_CTRLSTS_FUNC_SQRT     (9 << CORDIC_CTRLSTS_FUNC_SHIFT)

#define CORDIC_CTRLSTS_PRECISION_SHIFT 4
#define CORDIC_CTRLSTS_PRECISION_MASK  (0xf << CORDIC_CTRLSTS_PRECISION_SHIFT)

#define CORDIC_CTRLSTS_SCALE_SHIFT     8
#define CORDIC_CTRLSTS_SCALE_MASK      (0x7 << CORDIC_CTRLSTS_SCALE_SHIFT)

#define CORDIC_CTRLSTS_INTEN           (1 << 16)   /* Interrupt enable */
#define CORDIC_CTRLSTS_DMAREN          (1 << 17)   /* DMA read enable */
#define CORDIC_CTRLSTS_DMAWEN          (1 << 18)   /* DMA write enable */
#define CORDIC_CTRLSTS_NUMREAD         (1 << 19)   /* Number of results (1 = two, 0 = one) */
#define CORDIC_CTRLSTS_NUMWRITE        (1 << 20)   /* Number of arguments (1 = two, 0 = one) */
#define CORDIC_CTRLSTS_OUTSIZE         (1 << 21)   /* Output width: 0=32bit, 1=16bit */
#define CORDIC_CTRLSTS_INSIZE          (1 << 22)   /* Input width:  0=32bit, 1=16bit */
#define CORDIC_CTRLSTS_FLOATIN         (1 << 23)   /* Float input enable */
#define CORDIC_CTRLSTS_FLOATOUT        (1 << 24)   /* Float output enable */
#define CORDIC_CTRLSTS_PHASELIMIT      (1 << 25)   /* Phase limit enable */
#define CORDIC_CTRLSTS_CODINLIMIT      (1 << 26)   /* Coordinate limit enable */
#define CORDIC_CTRLSTS_INOVINTEN       (1 << 27)   /* Input overflow interrupt enable */
#define CORDIC_CTRLSTS_INOVF           (1 << 30)   /* Input overflow flag (write 1 to clear) */
#define CORDIC_CTRLSTS_RRF             (1 << 31)   /* Result ready flag */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_CORDIC_H */
