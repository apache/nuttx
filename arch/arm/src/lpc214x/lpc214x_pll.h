/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_pll.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_PLL_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL bass addresses *******************************************************/

/* There are two PLLs:
 * PLL0 generates CCLK and
 * PLL1 is configured to provide the 48MHx USB clock
 */

#define LPC214X_PLL0_BASE               (LPC214X_PLL_BASE)
#define LPC214X_PLL1_BASE               (LPC214X_PLL_BASE + 0x00000020)

/* PLL registers ************************************************************/

#define LPC214x_PLL0_CON                (LPC214X_PLL0_BASE+LPC214X_PLL_CON_OFFSET)
#define LPC214x_PLL0_CFG                (LPC214X_PLL0_BASE+LPC214X_PLL_CFG_OFFSET)
#define LPC214x_PLL0_STAT               (LPC214X_PLL0_BASE+LPC214X_PLL_STAT_OFFSET)
#define LPC214x_PLL0_FEED               (LPC214X_PLL0_BASE+LPC214X_PLL_FEED_OFFSET)

#define LPC214x_PLL1_CON                (LPC214X_PLL1_BASE+LPC214X_PLL_CON_OFFSET)
#define LPC214x_PLL1_CFG                (LPC214X_PLL1_BASE+LPC214X_PLL_CFG_OFFSET)
#define LPC214x_PLL1_STAT               (LPC214X_PLL1_BASE+LPC214X_PLL_STAT_OFFSET)
#define LPC214x_PLL1_FEED               (LPC214X_PLL1_BASE+LPC214X_PLL_FEED_OFFSET)

/* Register bit settings ****************************************************/

/* PLL Control Register Bit Settings */

#define LPC214X_PLL_CON_PLLE            (1 << 0) /* PLL Enable */
#define LPC214X_PLL_CON_PLLC            (1 << 1) /* PLL Connect */

/* PLL Configuration Register Bit Settings */

#define LPC214X_PLL_CFG_MSEL            (0x1f << 0) /* PLL Multiplier (minus 1) */
#define LPC214X_PLL_CFG_PSEL            (0x03 << 5) /* PLL Divider (encoded) */
#define LPC214X_PLL_CFG_PSEL1           (0x00 << 5)
#define LPC214X_PLL_CFG_PSEL2           (0x01 << 5)
#define LPC214X_PLL_CFG_PSEL4           (0x02 << 5)
#define LPC214X_PLL_CFG_PSEL8           (0x03 << 5)

/* PLL Status Register Bit Settings */

#define LPC214X_PLL_STAT_MSEL           (0x1f << 0) /* PLL Multiplier Readback */
#define LPC214X_PLL_STAT_PSEL           (0x03 << 5) /* PLL Divider Readback */
#define LPC214X_PLL_STAT_PLLE           (1 << 8)    /* PLL Enable Readback */
#define LPC214X_PLL_STAT_PLLC           (1 << 9)    /* PLL Connect Readback */
#define LPC214X_PLL_STAT_PLOCK          (1 << 10)   /* PLL Lock Status */

/* PLL Feed Register values */

#define LPC214X_PLL_FEED1               0xaa
#define LPC214X_PLL_FEED2               0x55

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_PLL_H */
