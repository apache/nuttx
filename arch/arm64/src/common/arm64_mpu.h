/****************************************************************************
 * arch/arm64/src/common/arm64_mpu.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_MPU_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convenience macros to represent the ARMv8-R64-specific configuration
 * for memory access permission and cache-ability attribution.
 */

/* MPU MPUIR Register Definitions */

#define MPU_IR_REGION_MSK      (0xFFU)

/* MPU RBAR Register attribute msk Definitions */

#define MPU_RBAR_BASE_POS   6U
#define MPU_RBAR_BASE_MSK   (0x3FFFFFFFFFFFFFFUL << MPU_RBAR_BASE_POS)
#define MPU_RBAR_SH_POS     4U
#define MPU_RBAR_SH_MSK     (0x3UL << MPU_RBAR_SH_POS)
#define MPU_RBAR_AP_POS     2U
#define MPU_RBAR_AP_MSK     (0x3UL << MPU_RBAR_AP_POS)

/* RBAR_EL1 XN */

#define MPU_RBAR_XN_POS     1U
#define MPU_RBAR_XN_MSK     (0x1UL << MPU_RBAR_XN_POS)

/* MPU PLBAR_ELx Register Definitions */

#define MPU_RLAR_LIMIT_POS     6U
#define MPU_RLAR_LIMIT_MSK     (0x3FFFFFFFFFFFFFFUL << MPU_RLAR_LIMIT_POS)
#define MPU_RLAR_ATTRINDX_POS  1U
#define MPU_RLAR_ATTRINDX_MSK  (0x7UL << MPU_RLAR_ATTRINDX_POS)
#define MPU_RLAR_EN_MSK        (0x1UL)

/* PRBAR_ELx: Attribute flag for not-allowing
 * execution (eXecute Never)
 */

#define NOT_EXEC      MPU_RBAR_XN_MSK /* PRBAR_EL1 */

/* PRBAR_ELx: Attribute flag for access permissions
 * Privileged Read Write, Unprivileged No Access
 */

#define P_RW_U_NA      0x0U
#define P_RW_U_NA_MSK  ((P_RW_U_NA << \
  MPU_RBAR_AP_POS) & MPU_RBAR_AP_MSK)

/* Privileged Read Write, Unprivileged Read Write */

#define P_RW_U_RW                       0x1U
#define P_RW_U_RW_MSK                   ((P_RW_U_RW << \
  MPU_RBAR_AP_POS) & MPU_RBAR_AP_MSK)

/* Privileged Read Only, Unprivileged No Access */

#define P_RO_U_NA                       0x2U
#define P_RO_U_NA_MSK                   ((P_RO_U_NA << \
  MPU_RBAR_AP_POS) & MPU_RBAR_AP_MSK)

/* Privileged Read Only, Unprivileged Read Only */

#define P_RO_U_RO                       0x3U
#define P_RO_U_RO_MSK                   ((P_RO_U_RO << \
  MPU_RBAR_AP_POS) & MPU_RBAR_AP_MSK)

/* PRBAR_ELx: Attribute flags for share-ability */

#define NON_SHAREABLE                   0x0U
#define NON_SHAREABLE_MSK \
  ((NON_SHAREABLE << MPU_RBAR_SH_POS) & MPU_RBAR_SH_MSK)
#define OUTER_SHAREABLE                 0x2U
#define OUTER_SHAREABLE_MSK \
  ((OUTER_SHAREABLE << MPU_RBAR_SH_POS) & MPU_RBAR_SH_MSK)
#define INNER_SHAREABLE                 0x3U
#define INNER_SHAREABLE_MSK \
  ((INNER_SHAREABLE << MPU_RBAR_SH_POS) & MPU_RBAR_SH_MSK)

/* MPIR_ELx Attribute flags for cache-ability */

/* Memory Attributes for Device Memory
 * 1.Gathering (G/nG)
 *   Determines whether multiple accesses can be merged into a single
 *   bus transaction.
 *   nG: Number/size of accesses on the bus = number/size of accesses
 *   in code.
 *
 * 2.Reordering (R/nR)
 *   Determines whether accesses to the same device can be reordered.
 *   nR: Accesses to the same IMPLEMENTATION DEFINED block size will
 *   appear on the bus in program order.
 *
 * 3 Early Write Acknowledgment (E/nE)
 *   Indicates to the memory system whether a buffer can send
 *   acknowledgements.
 *   nE: The response should come from the end slave, not buffering in
 *   the interconnect.
 */

#define DEVICE_nGnRnE       0x0U
#define DEVICE_nGnRE        0x4U
#define DEVICE_nGRE         0x8U
#define DEVICE_GRE          0xCU

/* Read/Write Allocation Configurations for Cacheable Memory
 * R_NON_W_NON:      Do not allocate Read/Write
 * R_NON_W_ALLOC:    Do not allocate Read, Allocate Write
 * R_ALLOC_W_NON:    Allocate Read, Do not allocate Write
 * R_ALLOC_W_ALLOC:  Allocate Read/Write
 */

#define R_NON_W_NON                     0x0U
#define R_NON_W_ALLOC                   0x1U
#define R_ALLOC_W_NON                   0x2U
#define R_ALLOC_W_ALLOC                 0x3U

/* Memory Attributes for Normal Memory
 * NORMAL_O_WT_NT: Normal, Outer Write-through on-transient
 * NORMAL_O_WB_NT: Normal, Outer Write-back non-transient
 * NORMAL_O_NON_C: Normal, Outer Non-Cacheable
 * NORMAL_I_WT_NT: Normal, Inner Write-through non-transient
 * NORMAL_I_WB_NT: Normal, Inner Write-back non-transient
 * NORMAL_I_NON_C: Normal, Inner Non-Cacheable
 */
#define NORMAL_O_WT_NT          0x80U
#define NORMAL_O_WB_NT          0xC0U
#define NORMAL_O_NON_C          0x40U
#define NORMAL_I_WT_NT          0x08U
#define NORMAL_I_WB_NT          0x0CU
#define NORMAL_I_NON_C          0x04U

/* Global MAIR configurations */

#define MPU_MAIR_INDEX_DEVICE           0U
#define MPU_MAIR_ATTR_DEVICE            (DEVICE_nGnRnE)

#define MPU_MAIR_INDEX_FLASH            1U
#define MPU_MAIR_ATTR_FLASH                  \
  ((NORMAL_O_WT_NT | (R_ALLOC_W_NON << 4)) | \
   (NORMAL_I_WT_NT | R_ALLOC_W_NON))

#define MPU_MAIR_INDEX_SRAM             2U
#define MPU_MAIR_ATTR_SRAM                     \
  ((NORMAL_O_WB_NT | (R_ALLOC_W_ALLOC << 4)) | \
   (NORMAL_I_WB_NT | R_ALLOC_W_ALLOC))

#define MPU_MAIR_INDEX_SRAM_NOCACHE     3U
#define MPU_MAIR_ATTR_SRAM_NOCACHE         \
  ((NORMAL_O_NON_C | (R_NON_W_NON << 4)) | \
   (NORMAL_I_NON_C | R_NON_W_NON))

#define MPU_MAIR_ATTRS                                     \
  ((MPU_MAIR_ATTR_DEVICE << (MPU_MAIR_INDEX_DEVICE * 8)) | \
   (MPU_MAIR_ATTR_FLASH << (MPU_MAIR_INDEX_FLASH * 8)) |   \
   (MPU_MAIR_ATTR_SRAM << (MPU_MAIR_INDEX_SRAM * 8)) |     \
   (MPU_MAIR_ATTR_SRAM_NOCACHE << (MPU_MAIR_INDEX_SRAM_NOCACHE * 8)))

/* Some helper defines for common regions.
 *
 * Note that the ARMv8-R MPU architecture requires that the
 * enabled MPU regions are non-overlapping. Therefore, it is
 * recommended to use these helper defines only for configuring
 * fixed MPU regions at build-time.
 */

#define REGION_DEVICE_ATTR                                \
  {                                                       \
    /* AP, XN, SH */                                      \
    .rbar = NOT_EXEC | P_RW_U_NA_MSK | NON_SHAREABLE_MSK, \
    /* Cache-ability */                                   \
    .mair_idx = MPU_MAIR_INDEX_DEVICE,                    \
  }

#define REGION_RAM_ATTR                                   \
  {                                                       \
    /* AP, XN, SH */                                      \
    .rbar = NOT_EXEC | P_RW_U_NA_MSK | NON_SHAREABLE_MSK, \
    /* Cache-ability */                                   \
    .mair_idx = MPU_MAIR_INDEX_SRAM,                      \
  }

#define REGION_RAM_TEXT_ATTR                   \
  {                                            \
    /* AP, XN, SH */                           \
    .rbar = P_RO_U_RO_MSK | NON_SHAREABLE_MSK, \
    /* Cache-ability */                        \
    .mair_idx = MPU_MAIR_INDEX_SRAM,           \
  }

#define REGION_RAM_RO_ATTR                                \
  {                                                       \
    /* AP, XN, SH */                                      \
    .rbar = NOT_EXEC | P_RO_U_RO_MSK | NON_SHAREABLE_MSK, \
    /* Cache-ability */                                   \
    .mair_idx = MPU_MAIR_INDEX_SRAM,                      \
  }

#ifndef __ASSEMBLY__

struct arm64_mpu_region_attr
{
  /* Attributes belonging to PRBAR */

  uint8_t rbar : 5;

  /* MAIR index for attribute indirection */

  uint8_t mair_idx : 3;
};

/* Region definition data structure */

struct arm64_mpu_region
{
  /* Region Base Address */

  uint64_t base;

  /* Region limit Address */

  uint64_t limit;

  /* Region Name */

  const char *name;

  /* Region Attributes */

  struct arm64_mpu_region_attr attr;
};

/* MPU configuration data structure */

struct arm64_mpu_config
{
  /* Number of regions */

  uint32_t num_regions;

  /* Regions */

  const struct arm64_mpu_region *mpu_regions;
};

#define MPU_REGION_ENTRY(_name, _base, _limit, _attr) \
  {                                                   \
    .name   = _name,                                  \
    .base   = _base,                                  \
    .limit  = _limit,                                 \
    .attr   = _attr,                                  \
  }

/* Reference to the MPU configuration.
 *
 * This struct is defined and populated for each SoC (in the SoC definition),
 * and holds the build-time configuration information for the fixed MPU
 * regions enabled during kernel initialization.
 */

extern const struct arm64_mpu_config g_mpu_config;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void arm64_mpu_init(bool is_primary_core);

#endif  /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM64_SRC_COMMON_ARM64_MPU_H */
