/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_pwm.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_PWM_H
#define __ARCH_ARM64_SRC_BCM2711_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base addresses */

#define BCM_PWM0_BASEADDR 0x7e20c000
#define BCM_PWM1_BASEADDR 0x7e20c800

/* PWM register offsets */

#define BCM_PWM_CTL_OFFSET 0x00
#define BCM_PWM_STA_OFFSET 0x04
#define BCM_PWM_DMAC_OFFSET 0x08
#define BCM_PWM_RNG1_OFFSET 0x10
#define BCM_PWM_DAT1_OFFSET 0x14
#define BCM_PWM_FIF1_OFFSET 0x18
#define BCM_PWM_RNG2_OFFSET 0x20
#define BCM_PWM_DAT2_OFFSET 0x24

/* PWM register addresses */

#define BCM_PWM0_CTL (BCM_PWM0_BASEADDR + BCM_PWM_CTL_OFFSET)
#define BCM_PWM0_STA (BCM_PWM0_BASEADDR + BCM_PWM_STA_OFFSET)
#define BCM_PWM0_DMAC (BCM_PWM0_BASEADDR + BCM_PWM_DMAC_OFFSET)
#define BCM_PWM0_RNG1 (BCM_PWM0_BASEADDR + BCM_PWM_RNG1_OFFSET)
#define BCM_PWM0_DAT1 (BCM_PWM0_BASEADDR + BCM_PWM_DAT1_OFFSET)
#define BCM_PWM0_FIF1 (BCM_PWM0_BASEADDR + BCM_PWM_FIF1_OFFSET)
#define BCM_PWM0_RNG2 (BCM_PWM0_BASEADDR + BCM_PWM_RNG2_OFFSET)
#define BCM_PWM0_DAT2 (BCM_PWM0_BASEADDR + BCM_PWM_DAT2_OFFSET)

#define BCM_PWM1_CTL (BCM_PWM0_BASEADDR + BCM_PWM_CTL_OFFSET)
#define BCM_PWM1_STA (BCM_PWM0_BASEADDR + BCM_PWM_STA_OFFSET)
#define BCM_PWM1_DMAC (BCM_PWM0_BASEADDR + BCM_PWM_DMAC_OFFSET)
#define BCM_PWM1_RNG1 (BCM_PWM0_BASEADDR + BCM_PWM_RNG1_OFFSET)
#define BCM_PWM1_DAT1 (BCM_PWM0_BASEADDR + BCM_PWM_DAT1_OFFSET)
#define BCM_PWM1_FIF1 (BCM_PWM0_BASEADDR + BCM_PWM_FIF1_OFFSET)
#define BCM_PWM1_RNG2 (BCM_PWM0_BASEADDR + BCM_PWM_RNG2_OFFSET)
#define BCM_PWM1_DAT2 (BCM_PWM0_BASEADDR + BCM_PWM_DAT2_OFFSET)

/* PWM register bit definitions */

#define BCM_PWM_CTL_MSEN2 (1 << 15)
#define BCM_PWM_CTL_USEF2 (1 << 13)
#define BCM_PWM_CTL_POLA2 (1 << 12)
#define BCM_PWM_CTL_SBIT2 (1 << 11)
#define BCM_PWM_CTL_RPTL2 (1 << 10)
#define BCM_PWM_CTL_MODE2 (1 << 9)
#define BCM_PWM_CTL_PWEN2 (1 << 8)
#define BCM_PWM_CTL_MSEN1 (1 << 7)
#define BCM_PWM_CTL_CLRF (1 << 6)
#define BCM_PWM_CTL_USEF1 (1 << 5)
#define BCM_PWM_CTL_POLA1 (1 << 4)
#define BCM_PWM_CTL_SBIT1 (1 << 3)
#define BCM_PWM_CTL_RPTL1 (1 << 2)
#define BCM_PWM_CTL_MODE1 (1 << 1)
#define BCM_PWM_CTL_PWEN1 (1 << 0)

#define BCM_PWM_STA_STA2 (1 << 10)
#define BCM_PWM_STA_STA1 (1 << 9)
#define BCM_PWM_STA_BERR (1 << 8)
#define BCM_PWM_STA_GAPO2 (1 << 5)
#define BCM_PWM_STA_GAPO1 (1 << 4)
#define BCM_PWM_STA_RERR1 (1 << 3)
#define BCM_PWM_STA_WERR1 (1 << 2)
#define BCM_PWM_STA_EMPT1 (1 << 1)
#define BCM_PWM_STA_FULL1 (1 << 0)

#define BCM_PWM_DMAC_ENAB (1 << 31)
#define BCM_PWM_DMAC_PANIC (0xff << 8)
#define BCM_PWM_DMAC_DREQ (0xff)

#endif /* __ARCH_ARM64_SRC_BCM2711_PWM_H */
