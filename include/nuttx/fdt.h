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
#include <stdbool.h>
#include <nuttx/compiler.h>

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

/****************************************************************************
 * Name: fdt_register
 *
 * Description:
 *   Store the pointer to the flattened device tree and verify that it at
 *   least appears to be valid. This function will not fully parse the FDT.
 *
 * Input Parameters:
 *  fdt_base - The pointer to the raw FDT.
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
 * Input Parameters:
 *   None
 *
 * Return:
 *   The pointer to the raw FDT.
 *
 ****************************************************************************/

FAR const char *fdt_get(void);

/****************************************************************************
 * Name: fdt_get_irq
 *
 * Description:
 *   Get the interrupt number of the node
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   nodeoffset - The offset of the node
 *   offset - The offset of the property
 *   irqbase - The base of the interrupt number
 *
 * Return:
 *   The interrupt number of the node
 *
 ****************************************************************************/

int fdt_get_irq(FAR const void *fdt, int nodeoffset,
                int offset, int irqbase);

/****************************************************************************
 * Name: fdt_get_irq_by_path
 *
 * Description:
 *   Get the interrupt number of the node
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *   path - The path of the node
 *   irqbase - The base of the interrupt number
 *
 * Return:
 *   The interrupt number of the node
 *
 ****************************************************************************/

int fdt_get_irq_by_path(FAR const void *fdt, int offset,
                        FAR const char *path, int irqbase);

/****************************************************************************
 * Name: fdt_get_parent_address_cells
 *
 * Description:
 *   Get the parent address of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *
 * Return:
 *   The parent address of the register space
 *
 ****************************************************************************/

int fdt_get_parent_address_cells(FAR const void *fdt, int offset);

/****************************************************************************
 * Name: fdt_get_parent_size_cells
 *
 * Description:
 *   Get the parent size of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *
 * Return:
 *   The parent size of the register space
 *
 ****************************************************************************/

int fdt_get_parent_size_cells(FAR const void *fdt, int offset);

/****************************************************************************
 * Name: fdt_ld_by_cells
 *
 * Description:
 *   Load a 32-bit or 64-bit value from a buffer, depending on the number
 *   of address cells.
 *
 * Input Parameters:
 *   value - The pointer to the buffer
 *   cells - The number of address cells
 *
 * Return:
 *   The 32-bit or 64-bit value
 *
 ****************************************************************************/

uintptr_t fdt_ld_by_cells(FAR const void *value, int cells);

/****************************************************************************
 * Name: fdt_get_reg_base_by_name
 *
 * Description:
 *   Get the value of the "reg" property by its offset in the "reg-names"
 *   property
 *
 * Input Parameters:
 *   fdt       - The pointer to the raw FDT.
 *   offset    - The offset to the node.
 *   reg_name  - The name of the register
 *
 * Returned Value:
 *   The register address determined by its name. Returns 0 if:
 *    - The reg-names property doesn't exist.
 *    - The reg property doesn't exits.
 *    - The reg-names property doesn't contain the "reg_name".
 *    - The offset combined with the size is larger than the width of the
 *      "reg" field
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base_by_name(FAR const void *fdt, int offset,
                                   FAR const char *reg_name);

/****************************************************************************
 * Name: fdt_get_reg_base
 *
 * Description:
 *   Get the base address of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *   index -  The index of the register in the reg field.
 *
 * Return:
 *   The base address of the register space
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base(FAR const void *fdt, int offset, int index);

/****************************************************************************
 * Name: fdt_get_reg_size
 *
 * Description:
 *   Get the size of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *
 * Return:
 *   The size of the register space
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_size(FAR const void *fdt, int offset);

/****************************************************************************
 * Name: fdt_get_reg_base_by_path
 *
 * Description:
 *   Get the base address of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   path - The path of the node
 *
 * Return:
 *   The base address of the register space
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base_by_path(FAR const void *fdt,
                                   FAR const char *path);

/****************************************************************************
 * Name: fdt_device_is_available
 *
 * Description:
 *   Test if node contains the "status" property with field set to okay or
 *   ok.
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset to the node to query.
 *
 * Returned Value:
 *   true:  The node contains the status propertry, and is set to okay or
 *          ok.
 *   false: The node contains the status propertry, but it is set to
 *          something other than ok or okay.
 *   Always returns true if the node doesn't contain a status property.
 *
 ****************************************************************************/

bool fdt_device_is_available(FAR const void * fdt, int offset);

/****************************************************************************
 * Name: fdt_get_node_label
 *
 * Description:
 *   Get the label for a given node. The device tree must be compiled with
 *   the -@ option in order for the symbol table to be generated.
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset to the node to query.
 *
 * Returned Value:
 *   Node label if found. NULL is returned if no label if found for the given
 *   node.
 *
 ****************************************************************************/

FAR const char *fdt_get_node_label(FAR const void *fdt, int offset);

/****************************************************************************
 * Name: fdt_get_clock_frequency
 *
 * Description:
 *   Get the value of the "clock-frequency" value for the given node.
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset to the node to query.
 *
 * Returned Value:
 *   The value of the clock-frequency property of the node. Zero is
 *   returned if the node doesn't contain a clock-frequency property.
 *
 ****************************************************************************/

uintptr_t fdt_get_clock_frequency(FAR const void *fdt, int offset);

/****************************************************************************
 * Name: fdt_get_clock_frequency_from_clocks
 *
 * Description:
 *   Get the "clock-frequency" property for the given node, using the phandle
 *   specified in the "clocks" property
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   node - The offset to the node to query.
 *   offset - The offset of the phandle in the clocks property
 *
 * Returned Value:
 *   The value of the clock-frequency property of the node, following the
 *   specified phandle in the "clocks"' property. Returns 0 if:
 *    - The node doesn't have a "clocks" property
 *    - The offset given is larger than the length of the "clocks" property
 *    - The phandle specified by the "clocks" property doesn't contain a
 *      "clock-frequency" property.
 *
 ****************************************************************************/

uintptr_t fdt_get_clock_frequency_from_clocks(FAR const void *fdt,
                                              int offset,
                                              int index);

/****************************************************************************
 * Name: fdt_node_index_from_label
 *
 * Description:
 *  Get the node index from a device tree label.
 *
 * Input Parameters:
 *  label - The device tree node_label
 *  count - The number of characters from the end of the label to search
 *
 * Returns
 *  The integer number found at the end of the label. e.g returns 4 for a
 *  label called (i2c_4). Returns -ENOENT if an integer cannot be found.
 *
 ****************************************************************************/

int fdt_node_index_from_label(FAR const char *node_label, int count);

/****************************************************************************
 * Name: fdt_node_from_compat
 *
 * Description:
 *   Find all devices with a matching compatibility string and call a device
 *   specific callback.
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   compatible_ids - NULL terminated list of compatible ids.
 *   driver_callback - Function called on every found instance of the ID.
 *
 ****************************************************************************/

void fdt_node_from_compat(FAR const void *fdt,
                          FAR const char **compatible_ids,
                          FAR void (*driver_callback)(FAR const void *fdt,
                                                      int offset));

/****************************************************************************
 * Name: fdt_load_prop_u32
 *
 * Description:
 *   Load an uint32_t type from a property at a given index
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The node offset
 *   property - The property to load from.
 *   index - The value index inside the property
 *   value - Output parameter for found value.
 *
 * Returns:
 *   OK on success, errno of failure.
 *
 ****************************************************************************/

int fdt_load_prop_u32(FAR const void *fdt, int offset,
                      FAR const char *property, int index,
                      FAR uint32_t *value);

/****************************************************************************
 * Name: pci_ecam_register_from_fdt
 *
 * Description:
 *   This function is used to register an ecam driver from the device tree
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

#ifdef CONFIG_PCI
int fdt_pci_ecam_register(FAR const void *fdt);
#endif

#endif /* __INCLUDE_NUTTX_FDT_H */
