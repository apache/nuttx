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
                        const char *path, int irqbase);

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
 * Name: fdt_get_reg_base
 *
 * Description:
 *   Get the base address of the register space
 *
 * Input Parameters:
 *   fdt - The pointer to the raw FDT.
 *   offset - The offset of the node
 *
 * Return:
 *   The base address of the register space
 *
 ****************************************************************************/

uintptr_t fdt_get_reg_base(FAR const void *fdt, int offset);

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

#endif /* __INCLUDE_NUTTX_FDT_H */
