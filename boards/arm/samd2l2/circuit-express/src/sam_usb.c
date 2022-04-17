/****************************************************************************
 * boards/arm/samd2l2/circuit-express/src/sam_usb.c
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
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "debug.h"
#include "chip.h"
#include "sam_port.h"
#include "saml_periphclks.h"
#include "circuit_express.h"

#if defined(CONFIG_SAMD2L2_USB)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_boot very early in initialization to setup
 *   USB-related GPIO pins for the Circuit Express board.
 *
 * USB Ports
 *   The SAML21 features USB device and host:
 *
 *
 ****************************************************************************/

void weak_function sam_usbinitialize(void)
{
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

#ifdef CONFIG_USBDEV
void sam_usb_suspend(struct usbdev_s *dev, bool resume)
{
  uinfo("board: resume: %d\n", resume);
}
#endif

#endif /* CONFIG_SAMDL_USB */
