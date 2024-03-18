/****************************************************************************
 * include/nuttx/fdt.h
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

#ifndef __INCLUDE_NUTTX_FDT_H
#define __INCLUDE_NUTTX_FDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/compiler.h>

#ifdef CONFIG_LIBC_FDT
#include <stdbool.h>
#include <stddef.h>
#endif /* CONFIG_LIBC_FDT */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FDT_MAGIC 0xd00dfeed

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fdt_header_s
{
  uint32_t magic;
  uint32_t totalsize;
  uint32_t off_dt_struct;
  uint32_t off_dt_strings;
  uint32_t off_mem_rsvmap;
  uint32_t version;
  uint32_t last_comp_version;
  uint32_t boot_cpuid_phys;
  uint32_t size_dt_strings;
  uint32_t size_dt_struct;
};

/****************************************************************************
 * Public Functions Definitions
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

const char *fdt_get_node_label(int node);

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

int fdt_get_size_cells(int node);

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

int fdt_get_address_cells(int node);

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

size_t fdt_get_clock_frequency_from_clocks(int node, int offset);

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

size_t fdt_get_clock_frequency(int node);

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

uintptr_t fdt_get_reg_by_name(int node, const char *reg_name);

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

uintptr_t fdt_get_reg(int node, int offset);

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

int fdt_get_irq(int node, int offset);

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

bool fdt_device_is_available(int node);
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

int fdt_register(FAR const char *fdt_base);

/****************************************************************************
 * Name: fdt_get
 *
 * Description:
 *   Return the pointer to a raw FDT. NULL is returned if no FDT has been
 *   loaded.
 *
 ****************************************************************************/

FAR const char *fdt_get(void);

#endif /* __INCLUDE_NUTTX_FDT_H */
