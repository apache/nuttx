/****************************************************************************
 * arch/arm/src/stm32h5/stm32_usbdrdhost.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_USBDRDHOST_H
#define __ARCH_ARM_SRC_STM32H5_STM32_USBDRDHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbhost.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Pre-requisites */

#if !defined(CONFIG_STM32H5_USBFS_HOST)
#  error "CONFIG_STM32H5_USBFS_HOST is required"
#endif

/* USB DRD Host Driver Configuration */

#ifndef CONFIG_STM32H5_USBDRD_NCHANNELS
#  define CONFIG_STM32H5_USBDRD_NCHANNELS 8
#endif

/* Default descriptor buffer size */

#ifndef CONFIG_STM32H5_USBDRD_DESCSIZE
#  define CONFIG_STM32H5_USBDRD_DESCSIZE 128
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the interface provided by the USB host controller
 * to the board-level USB host logic.
 */

struct stm32h5_usbhost_connection_s
{
  /* Wait for device connection/disconnection */

  struct usbhost_connection_s *conn;
};

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_usbhost_initialize
 *
 * Description:
 *   Initialize USB host controller hardware.
 *
 *   This function is called very early during system initialization to
 *   initialize the USB host controller. The USB host controller is then
 *   made available for use by the USB host driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the usbhost_connection_s interface is returned.
 *   On failure, NULL is returned.
 *
 ****************************************************************************/

struct usbhost_connection_s *stm32h5_usbhost_initialize(void);

/****************************************************************************
 * Name: stm32h5_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable VBUS power to the connected USB device.
 *
 *   The USB host driver calls this function during enumeration to control
 *   VBUS power delivery to the device. This function should be implemented
 *   by board-specific code since VBUS control is typically board-specific.
 *
 * Input Parameters:
 *   port   - USB host port number (0-based)
 *   enable - true: Enable VBUS power
 *            false: Disable VBUS power
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the USB host driver during enumeration.
 *
 ****************************************************************************/

void stm32h5_usbhost_vbusdrive(int port, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_USBDRDHOST_H */
