/****************************************************************************
 * boards/arm/stm32h7/nucleo-h723zg/src/stm32_usb.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_otg.h"
#include "nucleo-h723zg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBHOST)
#  error "USB host mode is not supported"
#endif
#if defined(CONFIG_USBDEV)
#  define HAVE_USB 1
#else
#  warning "CONFIG_STM32_OTGHS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the nucleo-144 board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void)
{
  /* The OTG HS has an internal soft pull-up.
   * No GPIO configuration is required
   */

  /* Configure the OTG HS VBUS sensing GPIO,
   * Power On, and Overcurrent GPIOs
   */

#ifdef CONFIG_STM32H7_OTGHS
  stm32_configgpio(GPIO_OTGHS_VBUS);
  stm32_configgpio(GPIO_OTGHS_PWRON);
  stm32_configgpio(GPIO_OTGHS_OVER);
#endif
}

/****************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV
 *   driver is used. This function is called whenever the USB enters or
 *   leaves suspend mode. This is an opportunity for the board logic to
 *   shutdown clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif
