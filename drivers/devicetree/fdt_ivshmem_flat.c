/****************************************************************************
 * drivers/devicetree/fdt_ivshmem_flat.c
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

#include <errno.h>

#include <nuttx/fdt.h>
#include <nuttx/ivshmem_flat.h>

#include <libfdt.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_ivshmem_flat_reg
 *
 * Description:
 *   Return reg[index] of the node as a CPU address, translated through the
 *   ranges of its parent, e.g. the QEMU platform bus. Returns 0 when no
 *   range covers it.
 *
 ****************************************************************************/

static uintptr_t fdt_ivshmem_flat_reg(FAR const void *fdt, int offset,
                                      int index)
{
  FAR const fdt32_t *ranges;
  uintptr_t addr;
  uintptr_t child;
  uintptr_t size;
  int parent;
  int len;
  int na;
  int pna;
  int ns;

  addr   = fdt_get_reg_base_by_index(fdt, offset, index);
  parent = fdt_parent_offset(fdt, offset);
  ranges = fdt_getprop(fdt, parent, "ranges", &len);
  if (ranges == NULL || len == 0)
    {
      return addr;
    }

  na  = fdt_address_cells(fdt, parent);
  ns  = fdt_size_cells(fdt, parent);
  pna = fdt_address_cells(fdt, fdt_parent_offset(fdt, parent));

  for (len /= sizeof(fdt32_t); len >= na + pna + ns;
       len -= na + pna + ns, ranges += na + pna + ns)
    {
      child = fdt_ld_by_cells(ranges, na);
      size  = fdt_ld_by_cells(ranges + na + pna, ns);
      if (addr >= child && addr - child < size)
        {
          return fdt_ld_by_cells(ranges + na, pna) + addr - child;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_ivshmem_flat_devices_register
 ****************************************************************************/

int fdt_ivshmem_flat_devices_register(FAR const void *fdt, int irqbase)
{
  struct ivshmem_flat_config_s config;
  uint32_t id;
  int offset = -1;
  int ret;

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset,
                                             "qemu,ivshmem-flat");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      config.reg        = fdt_ivshmem_flat_reg(fdt, offset, 0);
      config.shmem      = fdt_ivshmem_flat_reg(fdt, offset, 1);
      config.shmem_size = fdt_get_reg_size_by_index(fdt, offset, 1);
      if (config.reg == 0 || config.shmem == 0 || config.shmem_size == 0)
        {
          return -EINVAL;
        }

      /* No "interrupts" means no peer notification (memdev) */

      config.irq = fdt_get_irq(fdt, offset, 1, irqbase);

      /* "ivshmem-id" keeps the numbering independent of the order of
       * the -device options.
       */

      config.id = -1;
      if (fdt_load_prop_u32(fdt, offset, "ivshmem-id", 0, &id) == OK)
        {
          config.id = id;
        }

      ret = ivshmem_flat_register(&config);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
