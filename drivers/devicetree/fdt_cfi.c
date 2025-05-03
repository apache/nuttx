/****************************************************************************
 * drivers/devicetree/fdt_cfi.c
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
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/fdt.h>
#include <nuttx/mtd/mtd.h>

#include <libfdt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_cfi_register
 *
 * Description:
 *   This function is used to register an fci flash from the device tree
 *
 * Input Parameters:
 *   fdt - Device tree handle
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

int fdt_cfi_register(FAR const void *fdt)
{
  int ret = 0;
  int offset = -1;

  while ((offset = fdt_node_offset_by_compatible(fdt, offset, "cfi-flash"))
         >= 0)
    {
      uintptr_t address_cell = fdt_get_parent_address_cells(fdt, offset);
      uintptr_t size_cell    = fdt_get_parent_size_cells(fdt, offset);
      uint32_t bankwidth     = fdt_get_bankwidth(fdt, offset);
      uint32_t reg_cnt       = fdt_get_reg_count(fdt, offset);
      size_t num;
      size_t i;

      /* get number of "address & size" cell pairs */

      num = reg_cnt / (4 * (address_cell + size_cell));
      for (i = 0; i < num; i++)
        {
          uintptr_t flash_base = fdt_get_reg_base_by_index(fdt, offset, i);
          uintptr_t flash_end = fdt_get_reg_size_by_index(fdt, offset, i) +
                                flash_base;

          if ((CONFIG_FLASH_START > flash_end ||
               CONFIG_FLASH_START + CONFIG_FLASH_SIZE < flash_base))
            {
              ret = register_cfi_driver(flash_base, flash_end,
                                        bankwidth, i);
              if (ret < 0)
                {
                  break;
                }
            }
          else
            {
              ferr("cfi flash%d has been used to store text:"
                   "[%x,%x], flash:[%x,%x]\n", i, flash_base, flash_end,
                   CONFIG_FLASH_START, CONFIG_FLASH_START +
                   CONFIG_FLASH_SIZE);
            }
        }
    }

  return ret;
}
