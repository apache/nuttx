/****************************************************************************
 * drivers/devicetree/fdt_cmsdk_wdt.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/fdt.h>
#include <nuttx/timers/cmsdk_wdt.h>

#include <libfdt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fallback WDOGCLK if the device tree does not carry a resolvable rate. */

#define CMSDK_WDT_DEFAULT_FREQ 24000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Read a node's single-cell "clock-frequency" (Hz).  Falls back to
 * CMSDK_WDT_DEFAULT_FREQ when the property is absent, unreadable or zero.
 */

static uint32_t fdt_cmsdk_wdt_clock_hz(FAR const void *fdt, int offset)
{
  int len;
  FAR const fdt32_t *pv = fdt_getprop(fdt, offset, "clock-frequency", &len);
  uint32_t freq;

  if (pv == NULL || len < (int)sizeof(fdt32_t))
    {
      return CMSDK_WDT_DEFAULT_FREQ;
    }

  freq = fdt32_ld(pv);
  return freq != 0 ? freq : CMSDK_WDT_DEFAULT_FREQ;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_cmsdk_wdt_devices_register
 *
 * Description:
 *   Scan the device tree for "arm,cmsdk-wdt" nodes and register each as a
 *   watchdog character device (/dev/watchdog0, /dev/watchdog1, ...).
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *   irqbase  - Interrupt base number (added to FDT-supplied SPI index)
 *
 * Returned Value:
 *   Return OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int fdt_cmsdk_wdt_devices_register(FAR const void *fdt, int irqbase)
{
  uintptr_t addr;
  uintptr_t freq;
  char devpath[32];
  int offset = -1;
  int irqnum;
  int minor = 0;
  int ret;

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "arm,cmsdk-wdt");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      addr = fdt_get_reg_base(fdt, offset, 0);
      irqnum = fdt_get_irq(fdt, offset, 1, irqbase);
      if (addr == 0 || irqnum < 0)
        {
          return -EINVAL;
        }

      /* Read the single-cell "clock-frequency" directly (see fdt_sp804.c
       * for why the generic helper is unsuitable on this board); the helper
       * falls back to CMSDK_WDT_DEFAULT_FREQ when it is absent or zero.
       */

      freq = fdt_cmsdk_wdt_clock_hz(fdt, offset);

      snprintf(devpath, sizeof(devpath), "/dev/watchdog%d", minor++);
      ret = cmsdk_wdt_initialize(devpath, addr, irqnum, (uint32_t)freq);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
