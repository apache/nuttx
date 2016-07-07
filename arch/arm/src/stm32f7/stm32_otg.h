/************************************************************************************
 * arch/arm/src/stm32f7/stm32_otg.h
 *
 *   Copyright (C) 2012-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_OTG_H
#define __ARCH_ARM_SRC_STM32F7_STM32_OTG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "chip/stm32_otg.h"

#if defined(CONFIG_STM32F7_OTGFS) || defined(CONFIG_STM32F7_OTGHS)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifndef CONFIG_OTG_PRI
#  define CONFIG_OTG_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

#if defined(CONFIG_STM32F7_OTGFS)
#  define STM32_IRQ_OTG         STM32_IRQ_OTGFS
#  define STM32_OTG_BASE        STM32_USBOTGFS_BASE
#  define STM32_NENDPOINTS      (6)          /* ep0-5 x 2 for IN and OUT */
#  define GPIO_OTG_DM           GPIO_OTGFS_DM
#  define GPIO_OTG_DP           GPIO_OTGFS_DP
#  define GPIO_OTG_ID           GPIO_OTGFS_ID
#  define GPIO_OTG_SOF          GPIO_OTGFS_SOF
#  define STM32_OTG_FIFO_SIZE   1280
#endif

#if defined(CONFIG_STM32F7_OTGHS)
#  define STM32_IRQ_OTG         STM32_IRQ_OTGHS
#  define STM32_OTG_BASE        STM32_USBOTGHS_BASE
#  define STM32_NENDPOINTS      (7)          /* ep0-8 x 2 for IN and OUT but driver internals use byte to map + one bit for direction */
#  define GPIO_OTG_DM           GPIO_OTGHS_DM
#  define GPIO_OTG_DP           GPIO_OTGHS_DP
#  define GPIO_OTG_ID           GPIO_OTGHS_ID
#  define GPIO_OTG_SOF          GPIO_OTGHS_SOF
#  define STM32_OTG_FIFO_SIZE   4096
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

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
 * Name: stm32_otghost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initializeed.  Normally, this
 *     is just zero.
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
FAR struct usbhost_connection_s *stm32_otghost_initialize(int controller);
#endif

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the OTG FS device driver
 *   is used.  This function is called whenever the USB enters or leaves suspend
 *   mode. This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ************************************************************************************/

void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32F7_OTGFS */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_OTG_H */
