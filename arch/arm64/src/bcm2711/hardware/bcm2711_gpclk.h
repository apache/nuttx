/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_gpclk.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_GPCLK_H
#define __ARCH_ARM64_SRC_BCM2711_GPCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General purpose clock register offsets */

#define BCM_GPCLK_CM_GP0CTL_OFFSET 0x70
#define BCM_GPCLK_CM_GP0DIV_OFFSET 0x74
#define BCM_GPCLK_CM_GP1CTL_OFFSET 0x78
#define BCM_GPCLK_CM_GP1DIV_OFFSET 0x7c
#define BCM_GPCLK_CM_GP2CTL_OFFSET 0x80
#define BCM_GPCLK_CM_GP2DIV_OFFSET 0x84

/* General purpose clock registers */

#define BCM_GPCLK_CM_GP0CTL (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP0CTL_OFFSET)
#define BCM_GPCLK_CM_GP0DIV (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP0DIV_OFFSET)
#define BCM_GPCLK_CM_GP1CTL (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP1CTL_OFFSET)
#define BCM_GPCLK_CM_GP1DIV (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP1DIV_OFFSET)
#define BCM_GPCLK_CM_GP2CTL (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP2CTL_OFFSET)
#define BCM_GPCLK_CM_GP2DIV (BCM_GPCLK_BASEADDR + BCM_GPCLK_CM_GP2DIV_OFFSET)

/* General purpose clock bit definitions */

#define BCM_GPCLK_PASSWD 0x5a /* Clock manager password */

#define BCM_GPCLK_CM_CTL_PASSWD (0xff << 24) /* CLK manager password mask */
#define BCM_GPCLK_CM_CTL_MASH (0x3 << 9)     /* MASH control */
#define BCM_GPCLK_MASH_INTDIV (0 << 9)       /* Integer division */
#define BCM_GPCLK_MASH_STG1 (1 << 9)         /* One stage MASH */
#define BCM_GPCLK_MASH_STG2 (2 << 9)         /* Two stage MASH */
#define BCM_GPCLK_MASH_STG3 (3 << 9)         /* Three stage MASH */
#define BCM_GPCLK_CM_CTL_FLIP (1 << 8)       /* Invert CLK gen output */
#define BCM_GPCLK_CM_CTL_BUSY (1 << 7)       /* CLK gen running */
#define BCM_GPCLK_CM_CTL_KILL (1 << 5)       /* Kill CLK gen */
#define BCM_GPCLK_CM_CTL_ENAB (1 << 4)       /* Enable CLK gen */
#define BCM_GPCLK_CM_CTL_SRC (0xf)           /* Clock source */
#define BCM_GPCLK_CLKSRC_GND (0)             /* GND */
#define BCM_GPCLK_CLKSRC_OSC (1)             /* Oscillator */
#define BCM_GPCLK_CLKSRC_DBG0 (2)            /* testdebug0 */
#define BCM_GPCLK_CLKSRC_DBG1 (3)            /* testdebug1 */
#define BCM_GPCLK_CLKSRC_PLLA (4)            /* PLLA per */
#define BCM_GPCLK_CLKSRC_PLLC (5)            /* PLLC per */
#define BCM_GPCLK_CLKSRC_PLLD (6)            /* PLLD per */
#define BCM_GPCLK_CLKSRC_HDMI (7)            /* HDMI auxiliary */

#define BCM_GPCLK_CM_DIV_PASSWD (0xff << 24) /* CLK manager password mask */
#define BCM_GPCLK_CM_DIV_DIVI (0xfff << 12)  /* Integer part of divisor */
#define BCM_GPCLK_CM_DIV_DIVF (0xfff)        /* Fractional part of divisor */

#endif /* __ARCH_ARM64_SRC_BCM2711_GPCLK_H */
