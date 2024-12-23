/****************************************************************************
 * arch/tricore/src/common/tricore_mpu.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_MPU_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPU Region Attributes Bit Definitions */

#define REGION_TYPE_SHIFT   0
#define REGION_TYPE_MASK    (0x03 << REGION_TYPE_SHIFT)
#define REGION_TYPE_CODE    (0x01 << REGION_TYPE_SHIFT)
#define REGION_TYPE_DATA    (0x02 << REGION_TYPE_SHIFT)

#define REGION_ATTR_SHIFT   2
#define REGION_ATTR_MASK    (0x07 << REGION_ATTR_SHIFT)
#define REGION_ATTR_RE      (0x01 << REGION_ATTR_SHIFT)
#define REGION_ATTR_WE      (0x02 << REGION_ATTR_SHIFT)
#define REGION_ATTR_XE      (0x04 << REGION_ATTR_SHIFT)
#define REGION_ATTR_RO      REGION_ATTR_RE
#define REGION_ATTR_WO      REGION_ATTR_WE
#define REGION_ATTR_RW      (REGION_ATTR_RE | REGION_ATTR_WE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mpu_region_s
{
  uintptr_t base;
  size_t size;
  int kflags;
  int uflags;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocdataregions
 *
 * Description:
 *   Allocate data regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocdataregions(unsigned int nregions);

/****************************************************************************
 * Name: mpu_alloccoderegions
 *
 * Description:
 *   Allocate code regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_alloccoderegions(unsigned int nregions);

/****************************************************************************
 * Name: mpu_allocdataregion
 *
 * Description:
 *   Allocate data region
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

#define mpu_allocdataregion() mpu_allocdataregions(1)

/****************************************************************************
 * Name: mpu_alloccoderegion
 *
 * Description:
 *   Allocate code region
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

#define mpu_alloccoderegion() mpu_alloccoderegions(1)

/****************************************************************************
 * Name: mpu_freedataregion
 *
 * Description:
 *   Free data region
 *
 * Input Parameters:
 *   region - Region to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freedataregion(unsigned int region);

/****************************************************************************
 * Name: mpu_freecoderegion
 *
 * Description:
 *   Free code region
 *
 * Input Parameters:
 *   region - Region to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freecoderegion(unsigned int region);

/****************************************************************************
 * Name: mpu_allocregions
 *
 * Description:
 *   Allocate data or code regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *   flags    - Region flags.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocregions(unsigned int nregions, int flags);

/****************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *   Allocate the next region
 *
 * Input Parameters:
 *   flags    - Region flags.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

#define mpu_allocregion(flags) mpu_allocregions(1, flags)

/****************************************************************************
 * Name: mpu_freeregion
 *
 * Description:
 *   Free data region
 *
 * Input Parameters:
 *   region - Region to free.
 *   flags  - Region flags.
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freeregion(unsigned int region, int flags);

/****************************************************************************
 * Name: mpu_alloc_set
 *
 * Description:
 *   Allocate a protection set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The index of the allocated set.
 *
 ****************************************************************************/

unsigned int mpu_alloc_set(void);

/****************************************************************************
 * Name: mpu_free_set
 *
 * Description:
 *   Free a protection set
 *
 * Input Parameters:
 *   set - Set to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_free_set(unsigned int set);

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 * Input Parameters:
 *   enable - Flag indicating whether to enable the MPU.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_control(bool enable);

/****************************************************************************
 * Name: mpu_dump_set
 *
 * Description:
 *   Dump the regions of a protection set.
 *
 * Input Parameters:
 *   set - protection set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_dump_set(unsigned int set);

/****************************************************************************
 * Name: mpu_dump_regions
 *
 * Description:
 *   Dump the regions of all sets.
 *
 * Input Parameters:
 *   set - protection set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_dump_regions(void);

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region's attributes in the special protection set.
 *
 * Input Parameters:
 *   set        - Set number to modify.
 *   region     - Region number to modify.
 *   base       - Base address of the region.
 *   size       - Size of the region.
 *   flags      - Flags to configure the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int set, unsigned int region,
                       uintptr_t base, size_t size, int flags);

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region's attributes in the special protection set.
 *
 * Input Parameters:
 *   set        - Set number to modify.
 *   base       - Base address of the region.
 *   size       - Size of the region.
 *   flags      - Flags to configure the region.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(unsigned int set, uintptr_t base,
                                  size_t size, int flags);

/****************************************************************************
 * Name: mpu_get_active_set
 *
 * Description:
 *   Get the active protection set.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The active protection set.
 *
 ****************************************************************************/

unsigned int mpu_get_active_set(void);

/****************************************************************************
 * Name: mpu_set_active_set
 *
 * Description:
 *   Set the active protection set.
 *
 * Input Parameters:
 *   set - The protection set to activate.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_set_active_set(unsigned int set);

/****************************************************************************
 * Name: mpu_initialize
 *
 * Description:
 *   Initialize the MPU regions.
 *
 * Input Parameters:
 *   table      - MPU initialization table.
 *   count      - Initialize the number of entries in the region table.
 *
 * Returned Value:
 *   NULL.
 *
 ****************************************************************************/

void mpu_initialize(const struct mpu_region_s *table, size_t count);

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_MPU_H */
