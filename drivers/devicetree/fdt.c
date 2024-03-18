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

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <libfdt.h>
#include <nuttx/compiler.h>
#include <stddef.h>
#include <string.h>

#ifdef CONFIG_LIBC_FDT
#include <nuttx/fdt.h>
#endif /* CONFIG_LIBC_FDT */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Location of the fdt data for this system. */

static FAR const char *g_fdt_base = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_LIBC_FDT

/****************************************************************************
 * Name: fdt_get_node_label
 *
 * Description:
 *  Get the label for a given node. The device tree must be compiled with
 *  the -@ option in order for the symbol table to be generated.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *
 * Returned Value:
 *  Node label if found. NULL is returned if no label if found for the given
 *  node.
 *
 ****************************************************************************/

const char *fdt_get_node_label(int node)
{
  int symbols_offset;
  int property_offset;
  int ret;
  const char *node_name = NULL;
  const char *property_name;
  const char *label_name;
  char path_buffer[CONFIG_LIBC_FDT_PATH_MAX] =
    {
      0
    };

  DEBUGASSERT(g_fdt_base);

  symbols_offset = fdt_path_offset(g_fdt_base, "/__symbols__");
  if (symbols_offset < 0)
    {
      return node_name;
    }

  ret = fdt_get_path(g_fdt_base, node, path_buffer, sizeof(path_buffer));
  if (ret < 0)
    {
      return NULL;
    }

  fdt_for_each_property_offset(property_offset, g_fdt_base, symbols_offset)
    {
      property_name = fdt_getprop_by_offset(
          g_fdt_base, property_offset, &label_name, NULL);
      if (!strncmp(property_name, path_buffer, sizeof(path_buffer)))
        {
          return label_name;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: fdt_get_size_cells
 *
 * Description:
 *  Get the size cell count for the parent of the given node.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *
 * Returned Value:
 *  Numeric value of the size cells. Negative value returned on error.
 *
 ****************************************************************************/

int fdt_get_size_cells(int node)
{
  int parent_offset;

  DEBUGASSERT(g_fdt_base);

  parent_offset = fdt_parent_offset(g_fdt_base, node);
  if (parent_offset < 0)
    {
      return parent_offset;
    }

  return fdt_size_cells(g_fdt_base, parent_offset);
}

/****************************************************************************
 * Name: fdt_get_address_cells
 *
 * Description:
 *  Get the address cell count for the parent of the given node.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *
 * Returned Value:
 *  Numeric value of the address cells. Negative value returned on error.
 *
 ****************************************************************************/

int fdt_get_address_cells(int node)
{
  int parent_offset;

  DEBUGASSERT(g_fdt_base);

  parent_offset = fdt_parent_offset(g_fdt_base, node);
  if (parent_offset < 0)
    {
      return parent_offset;
    }

  return fdt_address_cells(g_fdt_base, parent_offset);
}

/****************************************************************************
 * Name: fdt_get_clock_frequency_from_clocks
 *
 * Description:
 *  Get the "clock-frequency" property for the given node, using the phandle
 *  specified in the "clocks" property
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *  offset: The offset the phandle in the clocks property
 *
 * Returned Value:
 *  The value of the clock-frequency property of the node, following the
 *  specified phandle in the "clocks"' property. Return 0 if:
 *   - The node doesn't have a "clocks" property
 *   - The offset given is larger than the length of the "clocks" property
 *   - The phandle specified by the "clocks" property doesn't contain a
 *     "clock-frequency" property.
 *
 ****************************************************************************/

size_t fdt_get_clock_frequency_from_clocks(int node, int offset)
{
  const fdt32_t *pv;
  fdt32_t clk_phandle;
  int pv_offset;
  uintptr_t clock_frequency = 0;
  int clk_length;

  DEBUGASSERT(g_fdt_base);

  pv = fdt_getprop(g_fdt_base, node, "clocks", &clk_length);
  if (!pv)
    {
      return clock_frequency;
    }

  if ((offset * sizeof(fdt32_t)) > clk_length)
    {
      return clock_frequency;
    }

  clk_phandle = fdt32_ld(pv + offset);

  pv_offset = fdt_node_offset_by_phandle(g_fdt_base, clk_phandle);
  if (pv_offset < 0)
    {
      return clock_frequency;
    }

  return fdt_get_clock_frequency(pv_offset);
}

/****************************************************************************
 * Name: fdt_get_clock_frequency
 *
 * Description:
 *  Get the value of the "clock-frequency" value for the given node.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *
 * Returned Value:
 *  The value of the clock-frequency property of the node. Zero is
 *  returnded if the node doesn't contain a clock-frequency property.
 *
 ****************************************************************************/

uintptr_t fdt_get_clock_frequency(int node)
{
  const void *pv;
  uintptr_t clock_frequency = 0;
  int size_cells;

  DEBUGASSERT(g_fdt_base);

  pv = fdt_getprop(g_fdt_base, node, "clock-frequency", NULL);
  if (!pv)
    {
      return clock_frequency;
    }

  size_cells = fdt_get_size_cells(node);
  if (size_cells < 0)
    {
      return clock_frequency;
    }

  if (size_cells == 2)
    {
      clock_frequency = fdt64_ld((fdt64_t *)pv);
    }
  else
    {
      clock_frequency = fdt32_ld((fdt32_t *)pv);
    }

  return clock_frequency;
}

/****************************************************************************
 * Name: fdt_get_reg_by_name
 *
 * Description:
 *  Get the value of the "reg" property by its offset in the "reg-names"
 *  property
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *  offset: The name of the register given in the "reg-names" property.
 *
 * Returned Value:
 *  The register address determined by its name. Returns 0 if:
 *   - The reg-names property doesn't exist.
 *   - The reg property doesn't exits.
 *   - The reg-names property doesn't contain the "reg_name".
 *   - The offset combined with the size is larger than the width of the
 *     "reg" field
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_by_name(int node, const char *reg_name)
{
  uintptr_t addr = 0;
  int reg_index;

  DEBUGASSERT(g_fdt_base);

  reg_index
      = fdt_stringlist_search(g_fdt_base, node, "reg-names", reg_name);
  if (reg_index < 0)
    {
      return addr;
    }

  return fdt_get_reg(node, reg_index);
}

/****************************************************************************
 * Name: fdt_get_reg
 *
 * Description:
 *  Get the value of the "reg" property at a specified offset.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *  offset: The offset of the register address inside the reg property.
 *
 * Returned Value:
 *  The register address at the given offset. Returns 0 if the "reg" property
 *  doesn't exist or the offset if larger than the size of the "reg"
 *  property.
 *
 ****************************************************************************/

uintptr_t fdt_get_reg(int node, int offset)
{
  const void *pv;
  uintptr_t addr = 0;
  int reg_length;
  int address_cells;

  DEBUGASSERT(g_fdt_base);

  /* Register cells contain a tuple of two values */

  offset *= 2;

  pv = fdt_getprop(g_fdt_base, node, "reg", &reg_length);
  if (pv != NULL)
    {
      if ((offset * sizeof(uintptr_t)) > reg_length)
        {
          return addr;
        }

      address_cells = fdt_get_address_cells(node);
      if (address_cells < 0)
        {
          return addr;
        }

      if (address_cells == 2)
        {
          addr = fdt64_ld((fdt64_t *)pv + offset);
        }
      else
        {
          addr = fdt32_ld((fdt32_t *)pv + offset);
        }
    }

  return addr;
}

/****************************************************************************
 * Name: fdt_get_irq
 *
 * Description:
 *  Get the numeric value of the "interrupts" property in the selected node.
 *
 * Input Parameters:
 *  node:   The offset to the node to query.
 *  offset: The offset of the interrupt number in the interrupts property.
 *
 * Returned Value:
 *  The interrupt number for the node. -ENOENT is return if the selected
 *  node doesn't contain an "interrupts" property.
 *
 ****************************************************************************/

int fdt_get_irq(int node, int offset)
{
  const fdt32_t *pv;
  int irq_number = -ENOENT;

  DEBUGASSERT(g_fdt_base);

  pv = fdt_getprop(g_fdt_base, node, "interrupts", NULL);
  if (pv != NULL)
    {
      irq_number = fdt32_ld(pv + offset);
    }

  return irq_number;
}

/****************************************************************************
 * Name: fdt_device_is_available
 *
 * Description:
 *  Test if node contains the "status" property with field set to okay or
 *ok.
 *
 * Input Parameters:
 *  node: The offset to the node to query.
 *
 * Returned Value:
 *  true:  The node contains the status propertry, and is set to okay or
 *         ok.
 *  false: The node contains the status propertry, but it is set to
 *         something other than ok or okay.
 *  Always returns true if the node doesn't contain a status property.
 *
 ****************************************************************************/

bool fdt_device_is_available(int node)
{
  DEBUGASSERT(g_fdt_base);

  const char *status = fdt_getprop(g_fdt_base, node, "status", NULL);
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
#endif /* CONFIG_LIBC_FDT */

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
