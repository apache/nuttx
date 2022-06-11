/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_usbdev.c
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>

#include "stm32_otgfs.h"
#include "viewtool_stm32f107.h"

#if defined(CONFIG_STM32_OTGFS) || defined(CONFIG_STM32_USB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbdev_initialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB related GPIO pins for the Viewtool STM32F107 board.
 *
 ****************************************************************************/

void stm32_usbdev_initialize(void)
{
  /* The OTG FS has an internal soft pull-up.
   * No GPIO configuration is required
   */

#ifdef CONFIG_ARCH_CHIP_STM32F103VC
  stm32_configgpio(GPIO_USB_PULLUP);
#endif
}

/****************************************************************************
 * Name:  stm32_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB
 *   software connect and disconnect), then the board software must provide
 *   stm32_pullup. See include/nuttx/usb/usbdev.h for additional
 *   description of this method. Alternatively, if no pull-up GPIO the
 *   following EXTERN can be redefined to be NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_STM32F103VC
int stm32_usbpullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);
  stm32_gpiowrite(GPIO_USB_PULLUP, !enable);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver
 *   is used.  This function is called whenever the USB enters or leaves
 *   suspend mode. This is an opportunity for the board logic to shutdown
 *   clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}

#endif /* CONFIG_STM32_OTGFS || CONFIG_STM32_USB*/
