/****************************************************************************
 * drivers/devicetree/fdt_imx_gpt.c
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
#include <inttypes.h>

#include <nuttx/fdt.h>
#include <nuttx/timers/imx_gpt.h>

#include <libfdt.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_imx_gpt_devices_register
 *
 * Description:
 *   Scan the device tree for "fsl,imx-gpt" nodes and bring each one up via
 *   imx_gpt_initialize(), registering its OCU channels as /dev/ocuN and its
 *   ICU channels as /dev/capN.
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *   irqbase  - Interrupt base number (added to the FDT-supplied SPI index)
 *
 * Returned Value:
 *   Return OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int fdt_imx_gpt_devices_register(FAR const void *fdt, int irqbase)
{
  uintptr_t addr;
  uint32_t freq;
  int offset = -1;
  int irqnum;
  int ocu_bus = 0;
  int cap_bus = 0;

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "fsl,imx-gpt");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      if (!fdt_device_is_available(fdt, offset))
        {
          continue;
        }

      addr   = fdt_get_reg_base(fdt, offset, 0);
      irqnum = fdt_get_irq(fdt, offset, 1, irqbase);

      /* clock-frequency is a single u32 cell.  Read it directly with
       * fdt_load_prop_u32() rather than fdt_get_clock_frequency(), which
       * decodes the value using the parent's #address-cells and would
       * mis-read the 32-bit scalar on a 2-address-cell machine.
       */

      freq = 0;
      fdt_load_prop_u32(fdt, offset, "clock-frequency", 0, &freq);
      if (addr == 0 || irqnum < 0)
        {
          continue;
        }

      if (imx_gpt_initialize(addr, irqnum, freq, ocu_bus, cap_bus) < 0)
        {
          syslog(LOG_ERR, "imx_gpt_initialize failed @0x%" PRIxPTR "\n",
                 addr);
          continue;
        }

#ifdef CONFIG_TIMER_IMX_GPT_OCU
      ocu_bus += IMX_GPT_NUM_OCU;
#endif
#ifdef CONFIG_TIMER_IMX_GPT_ICU
      cap_bus += IMX_GPT_NUM_ICU;
#endif
    }

  return OK;
}
