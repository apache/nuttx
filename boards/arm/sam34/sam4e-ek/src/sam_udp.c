/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_udp.c
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
#include "sam4e-ek.h"

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
 * Name:  sam_udp_suspend
 *
 * Description:
 *   Board logic must provide the sam_udp_suspend logic if the UDP driver is
 *   used.
 *   This function is called whenever the USB enters or leaves suspend mode.
 *
 *   When 'resume' is false, this function call provides an opportunity to
 *   perform board-specific power-saving actions so that less power is
 *   consumed while the USB is suspended.
 *
 *   Certain power-saving operations are performed by the UDP driver when
 *   it enters suspend mode:
 *   The USB device peripheral clocks are be switched off.  MCK and
 *   UDPCK are switched off and the USB transceiver is disabled.
 *
 *   When 'resume' is true, normal clocking and operations must all be
 *   restored.
 *
 ****************************************************************************/

void sam_udp_suspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
