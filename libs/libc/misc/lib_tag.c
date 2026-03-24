/****************************************************************************
 * libs/libc/misc/lib_tag.c
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

#include <stddef.h>
#include <sys/param.h>

#include <nuttx/tag.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kernel tag names -- order must match enum nxtag_e exactly */

static FAR const char *const g_nxtag_names[] =
{
  "analog",
  "audio",
  "battery",
  "binfmt",
  "can",
  "capture",
  "contactless",
  "crypto",
  "dma",
  "fs",
  "gpio",
  "graphics",
  "hwtracing",
  "i2c",
  "i2s",
  "i3c",
  "input",
  "ipc",
  "irq",
  "lcd",
  "leds",
  "lib",
  "memcard",
  "mm",
  "motor",
  "net",
  "paging",
  "pci",
  "power",
  "ptp",
  "pwm",
  "rc",
  "regmap",
  "reset",
  "rmt",
  "rpmsg",
  "rptun",
  "rtc",
  "safety",
  "sched",
  "sensors",
  "spi",
  "stepper",
  "syscall",
  "thermal",
  "timer",
  "usb",
  "vhost",
  "video",
  "virtio",
  "watchdog",
  "wireless",
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtag_get_name
 ****************************************************************************/

FAR const char *nxtag_get_name(uint8_t tag)
{
  if (tag < nitems(g_nxtag_names))
    {
      return g_nxtag_names[tag];
    }

  return NULL;
}
