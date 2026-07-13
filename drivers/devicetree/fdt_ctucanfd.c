/****************************************************************************
 * drivers/devicetree/fdt_ctucanfd.c
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
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/can/ctucanfd_pci.h>
#include <nuttx/fdt.h>

#include <libfdt.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ctucanfd_enable_on_this_cpu
 *
 * Description:
 *   Return true if the FDT node's cpu-affinity property matches the
 *   current CPU.  Nodes without the property are enabled on all
 *   cores (no affinity restriction); a present-but-mismatched property
 *   filters the node to its owning core only.  The former is logged as
 *   info (a node that will be enabled by every core), while the
 *   latter is normal multi-core filtering noise and stays silent.
 *
 ****************************************************************************/

static bool ctucanfd_enable_on_this_cpu(FAR const void *fdt, int offset)
{
  int len;
  FAR const fdt32_t *prop = fdt_getprop(fdt, offset, "cpu-affinity", &len);

  if (prop == NULL || len < (int)sizeof(fdt32_t))
    {
      caninfo("ctucanfd: FDT node at offset %d has no cpu-affinity "
              "property\n", offset);
      return true;
    }

  return fdt32_to_cpu(*prop) == up_cpu_index();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_ctucanfd_devices_register
 *
 * Description:
 *   Scan the device tree for nodes with compatible "ctu,ctucanfd" and
 *   register each via ctucanfd_register() (SysBus / MMIO path, no PCI).
 *
 *   The CAN channel number assigned to each node comes from the FDT
 *   can-id property when present (set by QEMU for ctucan-sysbus,
 *   supports non-contiguous / arbitrary channel numbering).  When the
 *   property is absent, the FDT discovery order (next) is used
 *   instead, preserving the legacy contiguous can0-8 numbering
 *   (compatible with the PCI path and older QEMU builds that do not
 *   emit can-id).
 *
 *   Each node's cpu-affinity property is checked against
 *   up_cpu_index(); only matching nodes are registered.
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *   irqbase  - Interrupt base number (added to FDT-supplied SPI index)
 *
 * Returned Value:
 *   Return OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int fdt_ctucanfd_devices_register(FAR const void *fdt, int irqbase)
{
  uintptr_t addr;
  int offset = -1;
  int irqnum;
  int ret;
  uint32_t next;
  uint32_t id;

  for (next = 0; ; next++)
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "ctu,ctucanfd");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      if (!ctucanfd_enable_on_this_cpu(fdt, offset))
        {
          continue;
        }

      /* Read the CAN channel number from the FDT can-id property.
       * When the property is present (set by QEMU for ctucan-sysbus),
       * it provides the global hardware channel number, supporting
       * non-contiguous numbering.  When absent, fall back to the FDT
       * discovery order index (compatible with PCI path and older
       * QEMU builds that do not emit can-id).
       */

      ret = fdt_load_prop_u32(fdt, offset, "can-id", 0, &id);
      if (ret != OK)
        {
          id = next;
        }

      addr = fdt_get_reg_base(fdt, offset, 0);
      irqnum = fdt_get_irq(fdt, offset, 1, irqbase);
      if (addr == 0 || irqnum < 0)
        {
          return -EINVAL;
        }

      ret = ctucanfd_register(addr, irqnum, id);
      if (ret < 0 && ret != -ENODEV)
        {
          return ret;
        }
    }

  return OK;
}
