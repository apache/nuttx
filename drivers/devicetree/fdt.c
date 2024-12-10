/****************************************************************************
 * drivers/devicetree/fdt.c
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
#include <endian.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>
#include <nuttx/compiler.h>
#include <nuttx/fdt.h>
#include <libfdt.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Location of the fdt data for this system. */

static FAR const char *g_fdt_base;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_register
 *
 * Description:
 *   Store the pointer to the flattened device tree and verify that it at
 *   least appears to be valid. This function will not fully parse the FDT.
 *
 * Return:
 *   Return -EINVAL if the fdt header does not have the expected magic value.
 *   otherwise return OK. If OK is not returned the existing entry for FDT
 *   is not modified.
 *
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

/****************************************************************************
 * Name: fdt_get
 *
 * Description:
 *   Return the pointer to a raw FDT. NULL is returned if no FDT has been
 *   loaded.
 *
 ****************************************************************************/

FAR const char *fdt_get(void)
{
  return g_fdt_base;
}

/****************************************************************************
 * Name: fdt_get_irq
 *
 * Description:
 *   Get the interrupt number of the node
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: fdt_get_irq_by_path
 *
 * Description:
 *   Get the interrupt number of the node
 *
 ****************************************************************************/

int fdt_get_irq_by_path(FAR const void *fdt, int offset,
                        const char *path, int irqbase)
{
  return fdt_get_irq(fdt, fdt_path_offset(fdt, path), offset, irqbase);
}

/****************************************************************************
 * Name: fdt_get_bankwidth
 *
 * Description:
 *   Get the value of bankwidth
 *
 ****************************************************************************/

uint32_t fdt_get_bankwidth(FAR const void *fdt, int offset)
{
  FAR const void *reg;
  uint32_t bankwidth = 0;

  reg = fdt_getprop(fdt, offset, "bank-width", NULL);
  if (reg != NULL)
    {
      bankwidth = fdt32_ld(reg);
    }

  return bankwidth;
}

/****************************************************************************
 * Name: fdt_get_parent_address_cells
 *
 * Description:
 *   Get the parent address of the register space
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: fdt_get_parent_size_cells
 *
 * Description:
 *   Get the parent size of the register space
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: fdt_ld_by_cells
 *
 * Description:
 *   Load a 32-bit or 64-bit value from a buffer, depending on the number
 *   of address cells.
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: fdt_get_reg_count
 *
 * Description:
 *   Get the count (in bytes) of the register space
 *
 ****************************************************************************/

uint32_t fdt_get_reg_count(FAR const void *fdt, int offset)
{
  FAR const struct fdt_property *reg;
  uint32_t count = 0;

  reg = fdt_get_property(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      count = fdt32_ld(&reg->len);
    }

  return count;
}

uintptr_t fdt_get_reg_base_by_name(FAR const void *fdt, int offset,
                                   FAR const char *reg_name)
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

/****************************************************************************
 * Name: fdt_get_reg_base
 *
 * Description:
 *   Get the base address of the register space
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: fdt_get_reg_base_by_index
 *
 * Description:
 *   Get the base address of the register space by index
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base_by_index(FAR const void *fdt, int offset,
                                    int index)
{
  FAR const void *reg;
  uintptr_t addr = 0;

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      int address_cell;
      int size_cell;

      address_cell = fdt_get_parent_address_cells(fdt, offset);
      size_cell = fdt_get_parent_size_cells(fdt, offset);
      addr = fdt_ld_by_cells((FAR fdt32_t *)reg +
                             (address_cell + size_cell) * index,
                             address_cell);
    }

  return addr;
}

/****************************************************************************
 * Name: fdt_get_reg_size
 *
 * Description:
 *   Get the size of the register space
 *
 ****************************************************************************/

size_t fdt_get_reg_size(FAR const void *fdt, int offset)
{
  FAR const void *reg;
  size_t size = 0;

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      size = fdt_ld_by_cells((FAR fdt32_t *)reg +
                             fdt_get_parent_address_cells(fdt, offset),
                             fdt_get_parent_size_cells(fdt, offset));
    }

  return size;
}

/****************************************************************************
 * Name: fdt_get_reg_size_by_index
 *
 * Description:
 *   Get the size of the register space by index
 *
 ****************************************************************************/

size_t fdt_get_reg_size_by_index(FAR const void *fdt, int offset, int index)
{
  FAR const void *reg;
  size_t size = 0;

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      int address_cell;
      int size_cell;

      address_cell = fdt_get_parent_address_cells(fdt, offset);
      size_cell = fdt_get_parent_size_cells(fdt, offset);
      size = fdt_ld_by_cells((FAR fdt32_t *)reg +
             (address_cell + size_cell) * index + address_cell, size_cell);
    }

  return size;
}

/****************************************************************************
 * Name: fdt_get_reg_base_by_path
 *
 * Description:
 *   Get the base address of the register space
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base_by_path(FAR const void *fdt, FAR const char *path)
{
  return fdt_get_reg_base(fdt, fdt_path_offset(fdt, path), 0);
}

bool fdt_device_is_available(FAR const void *fdt, int node)
{
  FAR const char *status = fdt_getprop(fdt, node, "status", NULL);
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

FAR const char *fdt_get_node_label(FAR const void *fdt, int node)
{
  int symbols_offset;
  int property_offset;
  int ret;
  const char *property_name;
  const char *label_name;
  char path_buffer[PATH_MAX] =
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

int fdt_node_index_from_label(FAR const char *node_label, int count)
{
  int dev_number = 0;
  size_t label_length;

  if (!node_label)
    {
      return -ENOENT;
    }

  label_length = strnlen(node_label, PATH_MAX);

  if (count > label_length || count <= 0)
    {
      return -EINVAL;
    }

  for (int i = 0; i < count; i++)
    {
      int number = atoi(&node_label[label_length - i]);
      if (number)
        {
          dev_number = number;
        }
    }

  /* atoi returns 0 on failure, so check that the number isn't actually 0 */

  if (!dev_number && !isdigit(node_label[label_length - 1]))
    {
      return -ENOENT;
    }

  return dev_number;
}

void fdt_node_from_compat(FAR const void *fdt,
                          FAR const char **compatible_ids,
                          FAR void (*driver_callback)(FAR const void *fdt,
                                                      int offset))
{
  int offset = 0;

  DEBUGASSERT(compatible_ids);
  DEBUGASSERT(driver_callback);

  while (*compatible_ids)
    {
      while (true)
        {
          offset
              = fdt_node_offset_by_compatible(fdt, offset, *compatible_ids);
          if (offset == -FDT_ERR_NOTFOUND)
            {
              break;
            }

          if (!fdt_device_is_available(fdt, offset))
            {
              continue;
            }

          driver_callback(fdt, offset);
        }

      compatible_ids++;
    }
}

int fdt_load_prop_u32(FAR const void *fdt, int offset,
                      FAR const char *property, int index,
                      FAR uint32_t *value)
{
  DEBUGASSERT(property);
  DEBUGASSERT(value);

  int length;
  const fdt32_t *pv = fdt_getprop(fdt, offset, property, &length);
  if (!pv)
    {
      return -ENOENT;
    }

  if (index >= length)
    {
      return -EINVAL;
    }

  *value = fdt32_ld(pv + index);
  return OK;
}
