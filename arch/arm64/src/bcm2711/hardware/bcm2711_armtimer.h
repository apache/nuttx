/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_armtimer.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_ARMTIMER_H
#define __ARCH_ARM64_SRC_BCM2711_ARMTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM timer register offsets */

#define BCM_ARMT_LOAD_OFFSET 0x400
#define BCM_ARMT_VALUE_OFFSET 0x404
#define BCM_ARMT_CONTROL_OFFSET 0x408
#define BCM_ARMT_IRQCNTL_OFFSET 0x40c
#define BCM_ARMT_RAWIRQ_OFFSET 0x410
#define BCM_ARMT_MSKIRQ_OFFSET 0x414
#define BCM_ARMT_RELOAD_OFFSET 0x418
#define BCM_ARMT_PREDIV_OFFSET 0x41c
#define BCM_ARMT_FREECNT_OFFSET 0x420

/* ARM timer register addresses */

#define _BCM_ARMT(offset) (BCM_ARMT_BASEADDR + (offset))

#define BCM_ARMT_LOAD _BCM_ARMT(BCM_ARMT_LOAD_OFFSET)
#define BCM_ARMT_VALUE _BCM_ARMT(BCM_ARMT_VALUE_OFFSET)
#define BCM_ARMT_CONTROL _BCM_ARMT(BCM_ARMT_CONTROL_OFFSET)
#define BCM_ARMT_IRQCNTL _BCM_ARMT(BCM_ARMT_IRQCNTL_OFFSET)
#define BCM_ARMT_RAWIRQ _BCM_ARMT(BCM_ARMT_RAWIRQ_OFFSET)
#define BCM_ARMT_MSKIRQ _BCM_ARMT(BCM_ARMT_MSKIRQ_OFFSET)
#define BCM_ARMT_RELOAD _BCM_ARMT(BCM_ARMT_RELOAD_OFFSET)
#define BCM_ARMT_PREDIV _BCM_ARMT(BCM_ARMT_PREDIV_OFFSET)
#define BCM_ARMT_FREECNT _BCM_ARMT(BCM_ARMT_FREECNT_OFFSET)

/* ARM timer register bit definitions */

#define BCM_ARMT_CONTROL_FREEDIV (0xff << 16)
#define BCM_ARMT_CONTROL_ENAFREE (1 << 9)
#define BCM_ARMT_CONTROL_DBGHALT (1 << 8)
#define BCM_ARMT_CONTROL_ENABLE (1 << 7)
#define BCM_ARMT_CONTROL_IE (1 << 6)
#define BCM_ARMT_CONTROL_DIV (0x3 << 2)
#define BCM_ARMT_CONTROL_DIVNONE (0 << 2)
#define BCM_ARMT_CONTROL_DIV16 (1 << 2)
#define BCM_ARMT_CONTROL_DIV256 (2 << 2)
#define BCM_ARMT_CONTROL_32BIT (1 << 1)

#define BCM_ARMT_IRQCNTL_INT (1 << 0)

#define BCM_ARMT_RAWIRQ_INT (1 << 0)

#define BCM_ARMT_MSKIRQ_INT (1 << 0)

#define BCM_ARMT_PREDIV_PREDIV (0x3ff << 0)

#endif /* __ARCH_ARM64_SRC_BCM2711_ARMTIMER_H */
