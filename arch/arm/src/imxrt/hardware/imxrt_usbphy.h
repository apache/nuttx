/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_usbphy.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USBPHY_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USBPHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_USBPHY_BASE                   (IMXRT_USBPHY1_BASE)  /* USB PHY Base */

/* Register Offsets *********************************************************/

#define IMXRT_USBPHY1_PWD_OFFSET            0x0000  /* USBPHY1 USB PHY Power-Down Register */
#define IMXRT_USBPHY1_PWD_CLR_OFFSET        0x0008  /* USBPHY1 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY1_CTRL_OFFSET           0x0030  /* USBPHY1 USB PHY General Control Register */
#define IMXRT_USBPHY1_CTRL_CLR_OFFSET       0x0038  /* USBPHY1 USB PHY General Control Register Clear */

#define IMXRT_USBPHY1_PLL_SIC_OFFSET        0x00a0
#define IMXRT_USBPHY1_PLL_SIC_SET_OFFSET    0x00a4
#define IMXRT_USBPHY1_PLL_SIC_CLR_OFFSET    0x00a8
#define IMXRT_USBPHY1_PLL_SIC_TOG_OFFSET    0x00ac

/* Register addresses *******************************************************/

#define IMXRT_USBPHY1_PWD                   (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PWD_OFFSET)      /* USBPHY1 USB PHY Power-Down Register */
#define IMXRT_USBPHY1_PWD_CLR               (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PWD_CLR_OFFSET)  /* USBPHY1 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY1_CTRL                  (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_CTRL_OFFSET)     /* USBPHY1 USB PHY General Control Register */
#define IMXRT_USBPHY1_CTRL_CLR              (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_CTRL_CLR_OFFSET) /* USBPHY1 USB PHY General Control Register Clear */
#define IMXRT_USBPHY1_PLL_SIC               (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PLL_SIC_OFFSET)
#define IMXRT_USBPHY1_PLL_SIC_SET           (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PLL_SIC_SET_OFFSET)
#define IMXRT_USBPHY1_PLL_SIC_CLR           (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PLL_SIC_CLR_OFFSET)
#define IMXRT_USBPHY1_PLL_SIC_TOG           (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PLL_SIC_TOG_OFFSET)

/* Register Bit Definitions *************************************************/

/* USB PHY Power-Down Register */

#define USBPHY_PWD_RXPWDRX          (1 << 20)  /* Bit 20: Power-down the entire USB PHY receiver block except for the full-speed differential receiver. */
#define USBPHY_PWD_RXPWDDIFF        (1 << 19)  /* Bit 19: Power-down the USB high-speed differential receiver. */
#define USBPHY_PWD_RXPWD1PT1        (1 << 18)  /* Bit 18: Power-down the USB full-speed differential receiver. */
#define USBPHY_PWD_RXPWDENV         (1 << 17)  /* Bit 17: Power-down the USB high-speed receiver envelope detector (squelch signal). */
#define USBPHY_PWD_TXPWDV2I         (1 << 12)  /* Bit 12: Power-down the USB PHY transmit V-to-I converter and the current mirror. */
#define USBPHY_PWD_TXPWDIBIAS       (1 << 11)  /* Bit 11: Power-down the USB PHY current bias block for the transmitter. */
#define USBPHY_PWD_TXPWDFS          (1 << 10)  /* Bit 10: Power-down the USB full-speed drivers. */

/* USB PHY General Control Register  */

#define USBPHY_CTRL_SFTRST          (1 << 31)  /* Bit 31: Soft-reset */
#define USBPHY_CTRL_CLKGATE         (1 << 30)  /* Bit 30: Gate UTMI clocks */

/* USB PHY PLL Control/Status Register (PLL_SIC, only in IMXRT117X) */
#define USBPHY1_PLL_SIC_PLL_POSTDIV_SHIFT  (2)        /* Bits 2-5: PLL_POSTDIV */
#define USBPHY1_PLL_SIC_PLL_POSTDIV_MASK   (0x7 << USBPHY1_PLL_SIC_PLL_POSTDIV_SHIFT)
#define USBPHY1_PLL_SIC_PLL_POSTDIV(n)     (((n) << USBPHY1_PLL_SIC_PLL_POSTDIV_SHIFT) & USBPHY1_PLL_SIC_PLL_POSTDIV_MASK)
#define USBPHY1_PLL_SIC_PLL_EN_USB_CLKS    (1 << 6)   /* Bit 6: PLL_EN_USB_CLKS */
#define USBPHY1_PLL_SIC_PLL_POWER          (1 << 12)  /* Bit 12: PLL_POWER */
#define USBPHY1_PLL_SIC_PLL_ENABLE         (1 << 13)  /* Bit 13: PLL_ENABLE */
#define USBPHY1_PLL_SIC_PLL_BYPASS         (1 << 16)  /* Bit 16: PLL_BYPASS */
#define USBPHY1_PLL_SIC_REFBIAS_PWD_SEL    (1 << 19)  /* Bit 19: REFBIAS_PWD_SEL */
#define USBPHY1_PLL_SIC_REFBIAS_PWD        (1 << 20)  /* Bit 20: Power down the reference bias */
#define USBPHY1_PLL_SIC_PLL_REG_ENABLE     (1 << 21)  /* Bit 21: PLL_REG_ENABLE */
#define USBPHY1_PLL_SIC_PLL_DIV_SEL_SHIFT  (22)       /* Bits 22-25: PLL_DIV_SEL */
#define USBPHY1_PLL_SIC_PLL_DIV_SEL_MASK   (0x7 << USBPHY1_PLL_SIC_PLL_DIV_SEL_SHIFT)
#define USBPHY1_PLL_SIC_PLL_DIV_SEL(n)     (((n) << USBPHY1_PLL_SIC_PLL_DIV_SEL_SHIFT) & USBPHY1_PLL_SIC_PLL_DIV_SEL_MASK)
#define USBPHY1_PLL_SIC_PLL_LOCK           (1 << 31)  /* Bit 31: PLL_LOCK */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USBPHY_H */
