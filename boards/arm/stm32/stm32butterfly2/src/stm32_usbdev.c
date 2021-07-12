/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_usbdev.c
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

#include <debug.h>
#include <stdbool.h>
#include <sys/boardctl.h>

#include "stm32_otgfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_OTGFS
#  error "CONFIG_USBDEV requires CONFIG_STM32_OTGFS to be enabled"
#endif

#ifdef CONFIG_USBHOST
#  error "CONFIG_USBDEV cannot be set alongside CONFIG_USBHOST"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver
 *   is used. This function is called whenever the USB enters or leaves
 *   suspend mode. This is an opportunity for the board logic to shutdown
 *   clocks, power, etc. while the USB is suspended.
 *
 * TODO:
 *   - Well... implement those features like clock shutdown.
 ****************************************************************************/

void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("INFO: usb %s", resume ? "resumed" : "suspended");
}
