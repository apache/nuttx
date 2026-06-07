/****************************************************************************
 * drivers/devicetree/fdt_pl061.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/fdt.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pl061.h>

#include <libfdt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PL061 has 8 GPIO lines per instance */

#define PL061_NPINS 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_pl061_devices_register
 *
 * Description:
 *   Scan the device tree for nodes with compatible "arm,pl061" and
 *   register each as an ioexpander via pl061_ioe_initialize(), then
 *   expose all 8 pins as standard /dev/gpioN character devices through
 *   gpio_lower_half().
 *
 *   Pins are initially registered as input (GPIO_INPUT_PIN); applications
 *   may switch them to output via ioctl(GPIOC_SETPINTYPE).
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *   irqbase  - Interrupt base number (added to FDT-supplied SPI index)
 *
 * Returned Value:
 *   Return OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int fdt_pl061_devices_register(FAR const void *fdt, int irqbase)
{
  FAR struct ioexpander_dev_s *dev;
  FAR const char *status;
  uintptr_t addr;
  int offset = -1;
  int irqnum;
#ifdef CONFIG_GPIO_LOWER_HALF
  int minor = 0;
  int i;
#endif

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "arm,pl061");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      /* Honor the standard device-tree 'status' property: skip nodes
       * marked "disabled" (i.e., not usable in this execution context).
       *
       * On the QEMU 'virt' machine there are two PL061 instances:
       *
       *   - Normal-world PL061 (@9030000):
       *       status = "okay"
       *       Used as general-purpose GPIO. pin 3 is the power-button
       *       input (gpio-keys), other pins are free.
       *
       *   - Secure-world PL061 (@90b0000):
       *       status         = "disabled"   <-- visible here, NS world
       *       secure-status  = "okay"       <-- only S world should use
       *       Wired internally to gpio-poweroff (pin 0) and gpio-restart
       *       (pin 1). Driving these pins HIGH from the normal world
       *       would shut down or reset the whole machine.
       *
       * Without this check we would register /dev/gpio0../dev/gpio7
       * against the secure controller, and the first ioctl(GPIOC_WRITE)
       * with value=1 on /dev/gpio0 would immediately power the VM off.
       */

      status = fdt_getprop(fdt, offset, "status", NULL);
      if (status != NULL && strcmp(status, "disabled") == 0)
        {
          continue;
        }

      addr = fdt_get_reg_base(fdt, offset, 0);
      irqnum = fdt_get_irq(fdt, offset, 1, irqbase);
      if (addr == 0 || irqnum < 0)
        {
          return -EINVAL;
        }

      dev = pl061_ioe_initialize(addr, irqnum);
      if (dev == NULL)
        {
          return -ENODEV;
        }

      /* Expose each pin as /dev/gpioN. Initial type is INPUT;
       * applications switch via ioctl(GPIOC_SETPINTYPE).
       *
       * Only available when the GPIO lower-half (CONFIG_GPIO_LOWER_HALF)
       * is enabled. Otherwise the PL061 is still registered as an
       * ioexpander above, just not exposed as /dev/gpioN char devices.
       */

#ifdef CONFIG_GPIO_LOWER_HALF
      for (i = 0; i < PL061_NPINS; i++)
        {
          gpio_lower_half(dev, i, GPIO_INPUT_PIN, minor++);
        }
#endif
    }

  return OK;
}
