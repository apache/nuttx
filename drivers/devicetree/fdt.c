/****************************************************************************
 * drivers/devicetree/fdt.c
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
#include <endian.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/compiler.h>
#include <nuttx/fdt.h>
#include <libfdt.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Location of the fdt data for this system. */

static FAR const char *g_fdt_base = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fdt_register(FAR const char *fdt_base)
{
  struct fdt_header_s *fdt_header;

  DEBUGASSERT(fdt_base);

  fdt_header = (struct fdt_header_s *)fdt_base;
  if (fdt_header->magic != be32toh(FDT_MAGIC))
    {
      return -EINVAL; /* Bad magic byte read */
    }

  g_fdt_base = fdt_base;
  return OK;
}

FAR const char *fdt_get(void)
{
  return g_fdt_base;
}

int fdt_get_irq(FAR const void *fdt, int nodeoffset,
                int offset, int irqbase)
{
  FAR const fdt32_t *pv;
  int irq = -1;

  pv = fdt_getprop(fdt, nodeoffset, "interrupts", NULL);
  if (pv != NULL)
    {
      irq = fdt32_ld(pv + offset) + irqbase;
    }

  return irq;
}

int fdt_get_irq_by_path(FAR const void *fdt, int offset,
                        const char *path, int irqbase)
{
  return fdt_get_irq(fdt, fdt_path_offset(fdt, path), offset, irqbase);
}

int fdt_get_parent_address_cells(FAR const void *fdt, int offset)
{
  int parentoff;

  parentoff = fdt_parent_offset(fdt, offset);
  if (parentoff < 0)
    {
      return parentoff;
    }

  return fdt_address_cells(fdt, parentoff);
}

int fdt_get_parent_size_cells(FAR const void *fdt, int offset)
{
  int parentoff;

  parentoff = fdt_parent_offset(fdt, offset);
  if (parentoff < 0)
    {
      return parentoff;
    }

  return fdt_size_cells(fdt, parentoff);
}

uintptr_t fdt_ld_by_cells(FAR const void *value, int cells)
{
  if (cells == 2)
    {
      return fdt64_ld(value);
    }
  else
    {
      return fdt32_ld(value);
    }
}

uintptr_t fdt_get_reg_base(FAR const void *fdt, int offset)
{
  FAR const void *reg;
  uintptr_t addr = 0;

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      addr = fdt_ld_by_cells(reg, fdt_get_parent_address_cells(fdt, offset));
    }

  return addr;
}

uintptr_t fdt_get_reg_size(FAR const void *fdt, int offset)
{
  FAR const void *reg;
  uintptr_t size = 0;

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      size = fdt_ld_by_cells(reg, fdt_get_parent_size_cells(fdt, offset));
    }

  return size;
}

uintptr_t fdt_get_reg_base_by_path(FAR const void *fdt, FAR const char *path)
{
  return fdt_get_reg_base(fdt, fdt_path_offset(fdt, path));
}

