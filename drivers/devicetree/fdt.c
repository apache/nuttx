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

uintptr_t fdt_get_reg_base_by_name(FAR const void *fdt, int offset,
                                   const char *reg_name)
{
  uintptr_t addr = 0;

  int reg_index
      = fdt_stringlist_search(fdt, offset, "reg-names", reg_name);
  if (reg_index < 0)
    {
      return addr;
    }

  return fdt_get_reg_base(fdt, offset, reg_index);
}

uintptr_t fdt_get_reg_base(FAR const void *fdt, int offset, int index)
{
  FAR const void *reg;
  uintptr_t addr = 0;
  int reg_length;

  /* Register cells contain a tuple of two values */

  index *= 2;

  reg = fdt_getprop(fdt, offset, "reg", &reg_length);
  if (reg != NULL)
    {
      if ((index * sizeof(uintptr_t)) > reg_length)
        {
          return addr;
        }

      addr = fdt_ld_by_cells(reg + index * sizeof(uintptr_t),
                             fdt_get_parent_address_cells(fdt, offset));
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
  return fdt_get_reg_base(fdt, fdt_path_offset(fdt, path), 0);
}

bool fdt_device_is_available(FAR const void *fdt, int node)
{
  const char *status = fdt_getprop(fdt, node, "status", NULL);
  if (!status)
    {
      return true;
    }

  if (!strcmp(status, "ok") || !strcmp(status, "okay"))
    {
      return true;
    }

  return false;
}

const char *fdt_get_node_label(FAR const void *fdt, int node)
{
  int symbols_offset;
  int property_offset;
  int ret;
  const char *property_name;
  const char *label_name;
  char path_buffer[CONFIG_PATH_MAX] =
    {
      0
    };

  symbols_offset = fdt_path_offset(fdt, "/__symbols__");
  if (symbols_offset < 0)
    {
      return NULL;
    }

  ret = fdt_get_path(fdt, node, path_buffer, sizeof(path_buffer));
  if (ret < 0)
    {
      return NULL;
    }

  fdt_for_each_property_offset(property_offset, fdt, symbols_offset)
    {
      property_name = fdt_getprop_by_offset(
          fdt, property_offset, &label_name, NULL);

      /* The symbols section is a list of parameters in the format
       * label_name = node_path. So the value of each property needs to be
       * checked with the full path found earlier.
       *
       */

      if (!strncmp(property_name, path_buffer, sizeof(path_buffer)))
        {
          return label_name;
        }
    }

  return NULL;
}

uintptr_t fdt_get_clock_frequency(FAR const void *fdt, int offset)
{
  const void *pv;
  uintptr_t clock_frequency = 0;

  pv = fdt_getprop(fdt, offset, "clock-frequency", NULL);
  if (!pv)
    {
      return clock_frequency;
    }

  clock_frequency = fdt_ld_by_cells(pv,
                                    fdt_get_parent_address_cells(fdt,
                                                                 offset));

  return clock_frequency;
}

uintptr_t fdt_get_clock_frequency_from_clocks(FAR const void *fdt,
                                              int offset,
                                              int index)
{
  const fdt32_t *pv;
  fdt32_t clk_phandle;
  int pv_offset;
  uintptr_t clock_frequency = 0;
  int clk_length;

  pv = fdt_getprop(fdt, offset, "clocks", &clk_length);
  if (!pv)
    {
      return clock_frequency;
    }

  if ((index * sizeof(fdt32_t)) > clk_length)
    {
      return clock_frequency;
    }

  clk_phandle = fdt32_ld(pv + index);

  pv_offset = fdt_node_offset_by_phandle(fdt, clk_phandle);
  if (pv_offset < 0)
    {
      return clock_frequency;
    }

  return fdt_get_clock_frequency(fdt, pv_offset);
}
