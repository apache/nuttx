/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_systimer.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_SYSTIMER_H
#define __ARCH_ARM64_SRC_BCM2711_SYSTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System timer register offsets */

#define BCM_SYST_CS_OFFSET 0x00
#define BCM_SYST_CLO_OFFSET 0x04
#define BCM_SYST_CHI_OFFSET 0x08
#define BCM_SYST_C0_OFFSET 0x0c
#define BCM_SYST_C1_OFFSET 0x10
#define BCM_SYST_C2_OFFSET 0x14
#define BCM_SYST_C3_OFFSET 0x18

/* System timer register addresses */

#define _BCM_SYST(offset) (BCM_SYST_BASEADDR + (offset))

#define BCM_SYST_CS _BCM_SYST(BCM_SYST_CS_OFFSET)
#define BCM_SYST_CLO _BCM_SYST(BCM_SYST_CLO_OFFSET)
#define BCM_SYST_CHI _BCM_SYST(BCM_SYST_CHI_OFFSET)
#define BCM_SYST_C0 _BCM_SYST(BCM_SYST_C0_OFFSET)
#define BCM_SYST_C1 _BCM_SYST(BCM_SYST_C1_OFFSET)
#define BCM_SYST_C2 _BCM_SYST(BCM_SYST_C2_OFFSET)
#define BCM_SYST_C3 _BCM_SYST(BCM_SYST_C3_OFFSET)

/* System timer register bit definitions */

#define BCM_SYST_CS_M3 (1 << 3)
#define BCM_SYST_CS_M2 (1 << 2)
#define BCM_SYST_CS_M1 (1 << 1)
#define BCM_SYST_CS_M0 (1 << 0)

#endif /* __ARCH_ARM64_SRC_BCM2711_SYSTIMER_H */
