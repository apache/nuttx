/****************************************************************************
 * arch/arm/src/armv8-m/mpu.h
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

#ifndef __ARCH_ARM_SRC_ARMV8M_MPU_H
#define __ARCH_ARM_SRC_ARMV8M_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#  include <assert.h>
#  include <debug.h>

#  include "arm_arch.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPU Register Bases */

#define MPU_BASE                0xe000ed90
#define MPU_BASE_NS             0xe002ed90

/* MPU Register Offsets */

#define MPU_TYPE_OFFSET         0x0000 /* MPU Type Register */
#define MPU_CTRL_OFFSET         0x0004 /* MPU Control Register */
#define MPU_RNR_OFFSET          0x0008 /* MPU Region Number Register */
#define MPU_RBAR_OFFSET         0x000c /* MPU Region Base Address Register */
#define MPU_RLAR_OFFSET         0x0010 /* MPU Region Limit Address Register */

#define MPU_RBAR_A1_OFFSET      0x0014 /* MPU alias registers */
#define MPU_RLAR_A1_OFFSET      0x0018
#define MPU_RBAR_A2_OFFSET      0x001c
#define MPU_RLAR_A2_OFFSET      0x0020
#define MPU_RBAR_A3_OFFSET      0x0024
#define MPU_RLAR_A3_OFFSET      0x0028

#define MPU_MAIR_OFFSET(n)      (0x0040 + 4 * ((n) >> 2))
#define MPU_MAIR0_OFFSET        0x0040 /* MPU Memory Attribute Indirection Register 0 */
#define MPU_MAIR1_OFFSET        0x0044 /* MPU Memory Attribute Indirection Register 1 */

/* MPU Register Addresses */

#define MPU_TYPE                (MPU_BASE + MPU_TYPE_OFFSET)
#define MPU_CTRL                (MPU_BASE + MPU_CTRL_OFFSET)
#define MPU_RNR                 (MPU_BASE + MPU_RNR_OFFSET)
#define MPU_RBAR                (MPU_BASE + MPU_RBAR_OFFSET)
#define MPU_RLAR                (MPU_BASE + MPU_RLAR_OFFSET)

#define MPU_RBAR_A1             (MPU_BASE + MPU_RBAR_A1_OFFSET)
#define MPU_RLAR_A1             (MPU_BASE + MPU_RLAR_A1_OFFSET)
#define MPU_RBAR_A2             (MPU_BASE + MPU_RBAR_A2_OFFSET)
#define MPU_RLAR_A2             (MPU_BASE + MPU_RLAR_A2_OFFSET)
#define MPU_RBAR_A3             (MPU_BASE + MPU_RBAR_A3_OFFSET)
#define MPU_RLAR_A3             (MPU_BASE + MPU_RLAR_A3_OFFSET)

#define MPU_MAIR(n)             (MPU_BASE + MPU_MAIR_OFFSET(n))
#define MPU_MAIR0               (MPU_BASE + MPU_MAIR0_OFFSET)
#define MPU_MAIR1               (MPU_BASE + MPU_MAIR1_OFFSET)

/* MPU Type Register Bit Definitions */

#define MPU_TYPE_SEPARATE       (1 << 0) /* Bit 0: 0:unified or 1:separate memory maps */
#define MPU_TYPE_DREGION_SHIFT  (8)      /* Bits 8-15: Number MPU data regions */
#define MPU_TYPE_DREGION_MASK   (0xff << MPU_TYPE_DREGION_SHIFT)
#define MPU_TYPE_IREGION_SHIFT  (16)     /* Bits 16-23: Number MPU instruction regions */
#define MPU_TYPE_IREGION_MASK   (0xff << MPU_TYPE_IREGION_SHIFT)

/* MPU Control Register Bit Definitions */

#define MPU_CTRL_ENABLE         (1 << 0)  /* Bit 0: Enable the MPU */
#define MPU_CTRL_HFNMIENA       (1 << 1)  /* Bit 1: Enable MPU during hard fault, NMI, and FAULTMAS */
#define MPU_CTRL_PRIVDEFENA     (1 << 2)  /* Bit 2: Enable privileged access to default memory map */

/* MPU Region Number Register Bit Definitions */

#if defined(CONFIG_ARM_MPU_NREGIONS) && defined(CONFIG_ARM_MPU)
#  if CONFIG_ARM_MPU_NREGIONS <= 8
#    define MPU_RNR_MASK            (0x00000007)
#  elif CONFIG_ARM_MPU_NREGIONS <= 16
#    define MPU_RNR_MASK            (0x0000000f)
#  elif CONFIG_ARM_MPU_NREGIONS <= 32
#    define MPU_RNR_MASK            (0x0000001f)
#  else
#    error "FIXME: Unsupported number of MPU regions"
#  endif
#endif

/* MPU Region Base Address Register Bit Definitions */

#define MPU_RBAR_XN             (1 << 0)                    /* Bit 0: Execute never */
#define MPU_RBAR_AP_SHIFT       (1)                         /* Bits 1-2: Access permission */
#define MPU_RBAR_AP_MASK        (3 << MPU_RBAR_AP_SHIFT)
#  define MPU_RBAR_AP_RWNO      (0 << MPU_RBAR_AP_SHIFT)    /* P:RW U:None */
#  define MPU_RBAR_AP_RWRW      (1 << MPU_RBAR_AP_SHIFT)    /* P:RW U:RW   */
#  define MPU_RBAR_AP_RONO      (2 << MPU_RBAR_AP_SHIFT)    /* P:RO U:None */
#  define MPU_RBAR_AP_RORO      (3 << MPU_RBAR_AP_SHIFT)    /* P:RO U:RO   */
#define MPU_RBAR_SH_SHIFT       (3)                         /* Bits 3-4: Shareability */
#define MPU_RBAR_SH_MASK        (3 << MPU_RBAR_SH_SHIFT)
#  define MPU_RBAR_SH_NO        (0 << MPU_RBAR_SH_SHIFT)    /* Non-shareable */
#  define MPU_RBAR_SH_OUTER     (2 << MPU_RBAR_SH_SHIFT)    /* Outer shareable */
#  define MPU_RBAR_SH_INNER     (3 << MPU_RBAR_SH_SHIFT)    /* Inner shareable */
#define MPU_RBAR_BASE_MASK      0xffffffe0                  /* Bits 5-31: Region base addrese */

/* MPU Region Region Limit Address Register Bit Definitions */

#define MPU_RLAR_ENABLE         (1 << 0)                    /* Bit 0: Region enable */
#define MPU_RLAR_INDX_SHIFT     (1)                         /* Bits 1-3: Attribute index */
#define MPU_RLAR_INDX_MASK      (7 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_STRONGLY_ORDER (0 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_DEVICE         (1 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_NONCACHEABLE   (2 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_WRITE_THROUGH  (3 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_WRITE_BACK     (4 << MPU_RLAR_INDX_SHIFT)
#define MPU_RLAR_PXN            (1 << 4)                    /* Bit 4: Privileged execute never */
#define MPU_RLAR_LIMIT_MASK     0xffffffe0                  /* Bits 5-31: Region limit address */

/* MPU Memory Attribute Indirection Register Bit Definitions */

#define MPU_MAIR_INNER_WA       (1 << 0) /* Bit 0: Inner write allocation */
#define MPU_MAIR_INNER_RA       (1 << 1) /* Bit 1: Inner read allocation */
#define MPU_MAIR_INNER_WB       (1 << 2) /* Bit 2: Inner write back */
#define MPU_MAIR_INNER_NT       (1 << 3) /* Bit 3: Inner non-transient */
#define MPU_MAIR_INNER_NC       (4 << 0) /* Bit 0-3: Inner non-cacheable */
#define MPU_MAIR_INNER_NGNRNE   (0 << 2) /* Bit 2-3: Inner nGnRnE */
#define MPU_MAIR_INNER_NGNRE    (1 << 2) /* Bit 2-3: Inner nGnRE */
#define MPU_MAIR_INNER_NGRE     (2 << 2) /* Bit 2-3: Inner nGRE */
#define MPU_MAIR_INNER_GRE      (3 << 2) /* Bit 2-3: Inner GRE */

#define MPU_MAIR_OUTER_WA       (1 << 4) /* Bit 4: Outer write allocation */
#define MPU_MAIR_OUTER_RA       (1 << 5) /* Bit 5: Outer read allocation */
#define MPU_MAIR_OUTER_WB       (1 << 6) /* Bit 6: Outer write back */
#define MPU_MAIR_OUTER_NT       (1 << 7) /* Bit 7: Outer non-transient */
#define MPU_MAIR_OUTER_DEVICE   (0 << 4) /* Bit 4-7: Outer device */
#define MPU_MAIR_OUTER_NC       (4 << 4) /* Bit 4-7: Outer non-cacheable */

#define MPU_MAIR_STRONGLY_ORDER (MPU_MAIR_OUTER_DEVICE | MPU_MAIR_INNER_NGNRNE)
#define MPU_MAIR_DEVICE         (MPU_MAIR_OUTER_DEVICE | MPU_MAIR_INNER_NGNRE)
#define MPU_MAIR_NONCACHEABLE   (MPU_MAIR_OUTER_NC | MPU_MAIR_INNER_NC)
#define MPU_MAIR_WRITE_THROUGH  (MPU_MAIR_OUTER_NT | MPU_MAIR_OUTER_RA | \
                                 MPU_MAIR_OUTER_WA | MPU_MAIR_INNER_NT | \
                                 MPU_MAIR_INNER_RA | MPU_MAIR_INNER_WA)
#define MPU_MAIR_WRITE_BACK     (MPU_MAIR_OUTER_NT | MPU_MAIR_OUTER_WB | \
                                 MPU_MAIR_OUTER_RA | MPU_MAIR_OUTER_WA | \
                                 MPU_MAIR_INNER_NT | MPU_MAIR_INNER_WB | \
                                 MPU_MAIR_INNER_RA | MPU_MAIR_INNER_WA)

/****************************************************************************
 * Name: mpu_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled during
 *   MPU initialization.
 *
 ****************************************************************************/

#if defined(CONFIG_MPU_RESET)
void mpu_reset(void);
#else
#  define mpu_reset() do { } while (0)
#endif

/****************************************************************************
 * Name: mpu_early_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled immediately
 *   after reset.
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_MPU_EARLY_RESET)
void mpu_early_reset(void);
#else
#  define mpu_early_reset() do { } while (0)
#endif

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 ****************************************************************************/

void mpu_control(bool enable, bool hfnmiena, bool privdefena);

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                          uint32_t flags1, uint32_t flags2);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_showtype
 *
 * Description:
 *   Show the characteristics of the MPU
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define mpu_showtype() \
    do \
      { \
        uint32_t regval = getreg32(MPU_TYPE); \
        sinfo("%s MPU Regions: data=%d instr=%d\n", \
          (regval & MPU_TYPE_SEPARATE) != 0 ? "Separate" : "Unified", \
          (regval & MPU_TYPE_DREGION_MASK) >> MPU_TYPE_DREGION_SHIFT, \
          (regval & MPU_TYPE_IREGION_MASK) >> MPU_TYPE_IREGION_SHIFT); \
    } while (0)
#else
#  define mpu_showtype() do { } while (0)
#endif

/****************************************************************************
 * Name: mpu_priv_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

#define mpu_priv_stronglyordered(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RWNO, \
                           MPU_RLAR_STRONGLY_ORDER | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_user_flash
 *
 * Description:
 *   Configure a region for user program flash
 *
 ****************************************************************************/

#define mpu_user_flash(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RORO, \
                           MPU_RLAR_WRITE_BACK); \
    } while (0)

/****************************************************************************
 * Name: mpu_priv_flash
 *
 * Description:
 *   Configure a region for privileged program flash
 *
 ****************************************************************************/

#define mpu_priv_flash(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RONO, \
                           MPU_RLAR_WRITE_BACK); \
    } while (0)

/****************************************************************************
 * Name: mpu_user_intsram
 *
 * Description:
 *   Configure a region as user internal SRAM
 *
 ****************************************************************************/

#define mpu_user_intsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_XN | \
                           MPU_RBAR_AP_RWRW, \
                           MPU_RLAR_NONCACHEABLE | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_priv_intsram
 *
 * Description:
 *   Configure a region as privileged internal SRAM
 *
 ****************************************************************************/

#define mpu_priv_intsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RWNO, \
                           MPU_RLAR_NONCACHEABLE | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_user_extsram
 *
 * Description:
 *   Configure a region as user external SRAM
 *
 ****************************************************************************/

#define mpu_user_extsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_XN | \
                           MPU_RBAR_AP_RWRW | \
                           MPU_RBAR_SH_OUTER, \
                           MPU_RLAR_WRITE_BACK | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_priv_extsram
 *
 * Description:
 *   Configure a region as privileged external SRAM
 *
 ****************************************************************************/

#define mpu_priv_extsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RWNO | \
                           MPU_RBAR_SH_OUTER, \
                           MPU_RLAR_WRITE_BACK | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged peripheral address space
 *
 ****************************************************************************/

#define mpu_peripheral(base, size) \
  do \
    { \
      /* Then configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_AP_RWNO, \
                           MPU_RLAR_DEVICE | \
                           MPU_RLAR_PXN); \
    } while (0)

/****************************************************************************
 * Name: mpu_user_peripheral
 *
 * Description:
 *   Configure a region as user peripheral address space
 *
 ****************************************************************************/

#define mpu_user_peripheral(base, size) \
  do \
    { \
      /* Then configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RBAR_XN | \
                           MPU_RBAR_AP_RWRW, \
                           MPU_RLAR_DEVICE | \
                           MPU_RLAR_PXN); \
    } while (0)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARM_MPU */
#endif /* __ARCH_ARM_SRC_ARMV8M_MPU_H */
