/****************************************************************************
 * arch/arm/src/stm32h7/stm32_otg.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_OTG_H
#define __ARCH_ARM_SRC_STM32H7_STM32_OTG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "hardware/stm32_otg.h"

#if defined(CONFIG_STM32H7_OTGFS) || defined(CONFIG_STM32H7_OTGHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_STM32H7_OTGFS)
#  define STM32_IRQ_OTG         STM32_IRQ_OTGFS
#  define STM32_OTG_BASE        STM32_OTGFS_BASE /* OTG FS */
#  define STM32_NENDPOINTS      (7)              /* ep0-8 x 2 for IN and OUT but driver internals use byte to map + one bit for direction */
#  define GPIO_OTG_DM           GPIO_OTGFS_DM
#  define GPIO_OTG_DP           GPIO_OTGFS_DP
#  define GPIO_OTG_ID           GPIO_OTGFS_ID
#  define GPIO_OTG_SOF          GPIO_OTGFS_SOF
#  define STM32_OTG_FIFO_SIZE   4096
#endif

#if defined(CONFIG_STM32H7_OTGHS)
#  define STM32_IRQ_OTG         STM32_IRQ_OTGHS
#  define STM32_OTG_BASE        STM32_OTGHS_BASE /* OTG HS/FS */
#  define STM32_NENDPOINTS      (7)              /* ep0-8 x 2 for IN and OUT but driver internals use byte to map + one bit for direction */
#  define GPIO_OTG_DM           GPIO_OTGHS_DM
#  define GPIO_OTG_DP           GPIO_OTGHS_DP
#  define GPIO_OTG_ID           GPIO_OTGHS_ID
#  define GPIO_OTG_SOF          GPIO_OTGHS_SOF
#  define STM32_OTG_FIFO_SIZE   4096
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_otgfshost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally,
 *     this is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
struct usbhost_connection_s;
struct usbhost_connection_s *stm32_otgfshost_initialize(int controller);
#endif

/****************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the OTG FS
 *   device driver is used.  This function is called whenever the USB enters
 *   or leaves suspend mode. This is an opportunity for the board logic to
 *   shutdown clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

struct usbdev_s;
void stm32_usbsuspend(struct usbdev_s *dev, bool resume);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32H7_OTGFS */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_OTG_H */
