/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_usb_phy.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_PHY_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_USBPHY_BASE_OFFSET            0x1000  /* USB PHY Base */
#define IMXRT_USBPHY_BASE                   (IMXRT_ANATOP_BASE + IMXRT_USBPHY_BASE_OFFSET)  /* USB PHY Base */

/* Register Offsets *********************************************************/

#define IMXRT_USBPHY1_PWD_OFFSET            0x0000  /* USBPHY1 USB PHY Power-Down Register */
#define IMXRT_USBPHY1_PWD_CLR_OFFSET        0x0008  /* USBPHY1 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY1_CTRL_OFFSET           0x0030  /* USBPHY1 USB PHY General Control Register */
#define IMXRT_USBPHY1_CTRL_CLR_OFFSET       0x0038  /* USBPHY1 USB PHY General Control Register Clear */

/* Register addresses *******************************************************/

#define IMXRT_USBPHY1_PWD                   (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PWD_OFFSET)  /* USBPHY1 USB PHY Power-Down Register */
#define IMXRT_USBPHY1_PWD_CLR               (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_PWD_CLR_OFFSET)  /* USBPHY1 USB PHY Power-Down Register Clear */
#define IMXRT_USBPHY1_CTRL                  (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_CTRL_OFFSET)  /* USBPHY1 USB PHY General Control Register */
#define IMXRT_USBPHY1_CTRL_CLR              (IMXRT_USBPHY_BASE + IMXRT_USBPHY1_CTRL_CLR_OFFSET)  /* USBPHY1 USB PHY General Control Register Clear */

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

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_PHY_H */
