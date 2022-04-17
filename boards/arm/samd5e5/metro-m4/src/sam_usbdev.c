/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_usbdev.c
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
#include "metro-m4.h"
#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins
 *
 ****************************************************************************/

void sam_usbinitialize(void)
{
  /* USB Soft Connect Pullup */
}

/****************************************************************************
 * Name:  sam_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB
 *   software connect and disconnect), then the board software must provide
 *   sam_pullup. See include/nuttx/usb/usbdev.h for additional description
 *   of this method. Alternatively, if no pull-up GPIO the following EXTERN
 *   can be redefined to be NULL.
 *
 ****************************************************************************/

int sam_usbpullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);
  return OK;
}

/****************************************************************************
 * Name:  sam_usbsuspend
 *
 * Description:
 *   Board logic must provide the sam_usbsuspend logic if the USBDEV driver
 *   is used. This function is called whenever the USB enters or leaves
 *   suspend mode. This is an opportunity for the board logic to shutdown
 *   clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

void sam_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
