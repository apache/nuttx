/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_usbdev.c
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "same70-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_usbinitialize
 *
 * Description:
 *   Called from sam_boardinitialize very early in initialization to setup
 *   USB-related GPIO pins for the SAME70-XPLD board.
 *
 ****************************************************************************/

void sam_usbinitialize(void)
{
  /* Initialize the VBUS enable signal to HI output in any event so that, by
   * default, VBUS power is not provided at the USB connector.
   */

  sam_configgpio(GPIO_VBUSON);
}

/****************************************************************************
 * Name:  sam_usbsuspend
 *
 * Description:
 *   Board logic must provide the sam_usbsuspend logic if the USBDEV driver
 *   is used.
 *   This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ****************************************************************************/

void sam_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
