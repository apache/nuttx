/****************************************************************************
 * boards/mips/jz4780/ci20/src/jz4780_bringup.c
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

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/signal.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#include "ci20.h"
#include "jz4780_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HDMI_CEC      (GPIO_MODE_DEVICE0 | GPIO_PORTF | GPIO_PIN23)
#define HDMI_SCL      (GPIO_MODE_DEVICE0 | GPIO_PORTF | GPIO_PIN24)
#define HDMI_SDA      (GPIO_MODE_DEVICE0 | GPIO_PORTF | GPIO_PIN25)

#define HDMI_POWER_EN (GPIO_MODE_OUTPUT1 | GPIO_PORTA | GPIO_PIN25)

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
 * Name: jz4780_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int jz4780_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at /proc: %d\n",
            ret);
    }
#endif

#if defined(CONFIG_USBHOST)

  ret = jz_usbhost_initialize();
  if (ret != OK)
    {
      _err("ERROR: Failed to start USB host services: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_VIDEO_FB

  /* Configure the DDC pins */

  jz4780_configgpio(HDMI_POWER_EN);
  jz4780_configgpio(HDMI_CEC);
  jz4780_configgpio(HDMI_SDA);
  jz4780_configgpio(HDMI_SCL);

  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      _err("ERROR: fb_register() failed: %d\n", ret);
    }
#endif

  return ret;
}
