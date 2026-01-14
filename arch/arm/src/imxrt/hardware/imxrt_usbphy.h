/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_usbphy.h
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

#define IMXRT_USBPHY_BASE_OFFSET            0x1000  /* USB1 PHY Base */

/* Simple hack to get iMXRT117x working with same macro */

#ifdef CONFIG_ARCH_FAMILY_IMXRT117x
#  define IMXRT_ANATOP_BASE                 0x40433000 /* ANATOP doesn't exist on rt117x, it is used this way here only to make the code compatible */
#  define IMXRT_USBPHY_SHIFT                0x4000
#else
#  define IMXRT_USBPHY_SHIFT                0x1000
#endif

#define IMXRT_USBPHY_BASE(n)                (IMXRT_ANATOP_BASE + IMXRT_USBPHY_BASE_OFFSET + (IMXRT_USBPHY_SHIFT * (n)))  /* USB PHY Base */

/* Register Offsets *********************************************************/

#define IMXRT_USBPHY_PWD_OFFSET             0x0000  /* USBPHY1/2 USB PHY Power-Down Register */
#define IMXRT_USBPHY_PWD_CLR_OFFSET         0x0008  /* USBPHY1/2 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY_CTRL_OFFSET            0x0030  /* USBPHY1/2 USB PHY General Control Register */
#define IMXRT_USBPHY_CTRL_CLR_OFFSET        0x0038  /* USBPHY1/2 USB PHY General Control Register Clear */

#define IMXRT_USBPHY_PLL_SIC_OFFSET         0x00a0
#define IMXRT_USBPHY_PLL_SIC_SET_OFFSET     0x00a4
#define IMXRT_USBPHY_PLL_SIC_CLR_OFFSET     0x00a8
#define IMXRT_USBPHY_PLL_SIC_TOG_OFFSET     0x00ac

/* Register addresses *******************************************************/

#define IMXRT_USBPHY_PWD(n)                   (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PWD_OFFSET)      /* USBPHY1 USB PHY Power-Down Register */
#define IMXRT_USBPHY_PWD_CLR(n)               (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PWD_CLR_OFFSET)  /* USBPHY1 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY_CTRL(n)                  (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_CTRL_OFFSET)     /* USBPHY1 USB PHY General Control Register */
#define IMXRT_USBPHY_CTRL_CLR(n)              (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_CTRL_CLR_OFFSET) /* USBPHY1 USB PHY General Control Register Clear */
#define IMXRT_USBPHY_PLL_SIC(n)               (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PLL_SIC_OFFSET)
#define IMXRT_USBPHY_PLL_SIC_SET(n)           (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PLL_SIC_SET_OFFSET)
#define IMXRT_USBPHY_PLL_SIC_CLR(n)           (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PLL_SIC_CLR_OFFSET)
#define IMXRT_USBPHY_PLL_SIC_TOG(n)           (IMXRT_USBPHY_BASE(n) + IMXRT_USBPHY_PLL_SIC_TOG_OFFSET)

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

#define USBPHY_CTRL_SFTRST            (1 << 31)  /* Bit 31: Soft-reset */
#define USBPHY_CTRL_CLKGATE           (1 << 30)  /* Bit 30: Gate UTMI clocks */
#define USBPHY_CTRL_UTMI_SUSPENDM     (1 << 29)  /* Bit 29: UTMI suspend DM */
#define USBPHY_CTRL_HOST_FORCE_LS_SE0 (1 << 28)  /* Bit 28: Host force low-speed */
#define USBPHY_CTRL_OTG_ID_VALUE      (1 << 27)  /* Bit 27: Filter glitches ID pad */
                                                 /* Bits 26-25: Reserved */
#define USBPHY_CTRL_FSDLL_RST_EN      (1 << 24)  /* Bit 24: Enable FSDLL logic */
#define USBPHY_CTRL_ENVBUSCHG_WKUP    (1 << 23)  /* Bit 23: Wakeup if VBUS change when USB is suspended */
#define USBPHY_CTRL_ENIDCHG_WKUP      (1 << 22)  /* Bit 22: Wakeup if ID change when USB is suspended */
#define USBPHY_CTRL_ENDPDMCHG_WKUP    (1 << 21)  /* Bit 21: Wakeup if DP/DM change when USB is suspended */
#define USBPHY_CTRL_ENAUTOCLR_PHY_PWD (1 << 20)  /* Bit 20: Autoclear PWD bit if there wakeup event when USB is suspended */
#define USBPHY_CTRL_ENAUTOCLR_CLKGATE (1 << 19)  /* Bit 19: Autoclear CLKGATE bit if there wakeup event when USB is suspended */
#define USBPHY_CTRL_ENAUTO_PWRON_PLL  (1 << 18)  /* Bit 18: Autoclear POWER bit if there wakeup event when USB is suspended */
#define USBPHY_CTRL_WAKEUP_IRQ        (1 << 17)  /* Bit 17: Indicates there is a wakeup event, reset this bit writing 1 to it */
#define USBPHY_CTRL_ENIRQWAKEUP       (1 << 16)  /* Bit 16: Enable interrupt for  wakeup event */
#define USBPHY_CTRL_ENUTMILEVEL3      (1 << 15)  /* Bit 15: Enable UTMI+ Level3 */
#define USBPHY_CTRL_ENUTMILEVEL2      (1 << 14)  /* Bit 14: Enable UTMI+ Level2 */
#define USBPHY_CTRL_DATA_ON_LRADC     (1 << 13)  /* Bit 13: Enable LRADC to monitor DP/DM, non-USB modes only */
#define USBPHY_CTRL_DEVPLUGIN_IRQ     (1 << 12)  /* Bit 12: Enable interrupt for detection activity to the USB line */
#define USBPHY_CTRL_ENIRQDEVPLUGIN    (1 << 11)  /* Bit 11: Enable interrupt for detection activity to the USB line */
#define USBPHY_CTRL_RESUME_IRQ        (1 << 10)  /* Bit 10: Indicates to host is sending a wake-up after suspend, write 1 to clear this interrupt */
#define USBPHY_CTRL_ENIRQRESUMEDET    (1 << 9)   /* Bit 9:  Enable the resume IRQ */
#define USBPHY_CTRL_RESUMEIRQSTICKY   (1 << 8)   /* Bit 8:  Set 1 to make RESUME IRQ Sticky */
#define USBPHY_CTRL_ENOTGIDDETECT     (1 << 7)   /* Bit 7:  Enable circuit to detect ID pin */
#define USBPHY_CTRL_OTG_ID_CHG_IRG    (1 << 6)   /* Bit 6:  OTG ID IRQ, indicates value of ID pin changed */
#define USBPHY_CTRL_DEVPLUGIN_POL     (1 << 5)   /* Bit 5:  If 0 IRQ when plugged in, if 1 IRQ when plugged out */
#define USBPHY_CTRL_ENDEVPLUGINDETECT (1 << 4)   /* Bit 4:  Enables 220K ohm pullup to detect host connectivity */
#define USBPHY_CTRL_HOSTDISCONDET_IRQ (1 << 3)   /* Bit 3:  Indicates the device was disconnected in high-speed mode, write 1 to clear IRQ */
#define USBPHY_CTRL_ENIRQHOSTDISCON   (1 << 2)   /* Bit 2:  Enable device disconnected IRQ */
#define USBPHY_CTRL_ENHOSTDISCONDET   (1 << 1)   /* Bit 1:  Enable HighSpeed disconnect detector */
#define USBPHY_CTRL_ENOTG_ID_CHG_IRQ  (1 << 0)   /* Bit 0:  Enable OTG_ID_CHG_IRQ */

/* USB PHY PLL Control/Status Register (PLL_SIC, only in IMXRT117X) */
#define USBPHY_PLL_SIC_PLL_POSTDIV_SHIFT   (2)        /* Bits 2-5: PLL_POSTDIV */
#define USBPHY_PLL_SIC_PLL_POSTDIV_MASK    (0x7 << USBPHY_PLL_SIC_PLL_POSTDIV_SHIFT)
#define USBPHY_PLL_SIC_PLL_POSTDIV(n)      (((n) << USBPHY_PLL_SIC_PLL_POSTDIV_SHIFT) & USBPHY_PLL_SIC_PLL_POSTDIV_MASK)
#define USBPHY_PLL_SIC_PLL_EN_USB_CLKS     (1 << 6)   /* Bit 6: PLL_EN_USB_CLKS */
#define USBPHY_PLL_SIC_PLL_POWER           (1 << 12)  /* Bit 12: PLL_POWER */
#define USBPHY_PLL_SIC_PLL_ENABLE          (1 << 13)  /* Bit 13: PLL_ENABLE */
#define USBPHY_PLL_SIC_PLL_BYPASS          (1 << 16)  /* Bit 16: PLL_BYPASS */
#define USBPHY_PLL_SIC_REFBIAS_PWD_SEL     (1 << 19)  /* Bit 19: REFBIAS_PWD_SEL */
#define USBPHY_PLL_SIC_REFBIAS_PWD         (1 << 20)  /* Bit 20: Power down the reference bias */
#define USBPHY_PLL_SIC_PLL_REG_ENABLE      (1 << 21)  /* Bit 21: PLL_REG_ENABLE */
#define USBPHY_PLL_SIC_PLL_DIV_SEL_SHIFT   (22)       /* Bits 22-25: PLL_DIV_SEL */
#define USBPHY_PLL_SIC_PLL_DIV_SEL_MASK    (0x7 << USBPHY_PLL_SIC_PLL_DIV_SEL_SHIFT)
#define USBPHY_PLL_SIC_PLL_DIV_SEL(n)      (((n) << USBPHY_PLL_SIC_PLL_DIV_SEL_SHIFT) & USBPHY_PLL_SIC_PLL_DIV_SEL_MASK)
#define USBPHY_PLL_SIC_PLL_LOCK            (1 << 31)  /* Bit 31: PLL_LOCK */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USBPHY_H */
