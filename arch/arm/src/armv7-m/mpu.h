/*********************************************************************************************
 * arch/arm/src/armv7-m/mpu.h
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
 *********************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7M_MPU_H
#define __ARCH_ARM_SRC_ARMV7M_MPU_H

/*********************************************************************************************
 * Included Files
 *********************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#  include <assert.h>
#  include <debug.h>

#  include "arm_arch.h"
#endif

/*********************************************************************************************
 * Pre-processor Definitions
 *********************************************************************************************/

/* MPU Register Addresses */

#define MPU_TYPE                0xe000ed90 /* MPU Type Register */
#define MPU_CTRL                0xe000ed94 /* MPU Control Register */
#define MPU_RNR                 0xe000ed98 /* MPU Region Number Register */
#define MPU_RBAR                0xe000ed9c /* MPU Region Base Address Register */
#define MPU_RASR                0xe000eda0 /* MPU Region Attribute and Size Register */

#define MPU_RBAR_A1             0xe000eda4 /* MPU alias registers */
#define MPU_RASR_A1             0xe000eda8
#define MPU_RBAR_A2             0xe000edac
#define MPU_RASR_A2             0xe000edb0
#define MPU_RBAR_A3             0xe000edb4
#define MPU_RASR_A3             0xe000edb8

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

#define MPU_RBAR_REGION_SHIFT   (0)                         /* Bits 0-3: MPU region */
#define MPU_RBAR_REGION_MASK    (15 << MPU_RBAR_REGION_SHIFT)
#define MPU_RBAR_VALID          (1 << 4)                    /* Bit 4: MPU Region Number valid */
#define MPU_RBAR_ADDR_MASK      0xffffffe0                  /* Bits N-31:  Region base addrese */

/* MPU Region Attributes and Size Register Bit Definitions */

#define MPU_RASR_ENABLE         (1 << 0)                   /* Bit 0: Region enable */
#define MPU_RASR_SIZE_SHIFT     (1)                        /* Bits 1-5: Size of the MPU protection region */
#define MPU_RASR_SIZE_MASK      (31 << MPU_RASR_SIZE_SHIFT)
#  define MPU_RASR_SIZE_LOG2(n) ((n-1) << MPU_RASR_SIZE_SHIFT)
#define MPU_RASR_SRD_SHIFT      (8)                        /* Bits 8-15: Subregion disable */
#define MPU_RASR_SRD_MASK       (0xff << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_0        (0x01 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_1        (0x02 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_2        (0x04 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_3        (0x08 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_4        (0x10 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_5        (0x20 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_6        (0x40 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_7        (0x80 << MPU_RASR_SRD_SHIFT)
#define MPU_RASR_ATTR_SHIFT     (16)                       /* Bits 16-31: MPU Region Attribute field */
#define MPU_RASR_ATTR_MASK      (0xffff << MPU_RASR_ATTR_SHIFT)
#  define MPU_RASR_B            (1 << 16)                  /* Bit 16: Bufferable */
#  define MPU_RASR_C            (1 << 17)                  /* Bit 17: Cacheable */
#  define MPU_RASR_S            (1 << 18)                  /* Bit 18: Shareable */
#  define MPU_RASR_TEX_SHIFT    (19)                       /* Bits 19-21: TEX Address Permission */
#  define MPU_RASR_TEX_MASK     (7 << MPU_RASR_TEX_SHIFT)
#    define MPU_RASR_TEX_SO     (0 << MPU_RASR_TEX_SHIFT) /* Strongly Ordered */
#    define MPU_RASR_TEX_NOR    (1 << MPU_RASR_TEX_SHIFT) /* Normal           */
#    define MPU_RASR_TEX_DEV    (2 << MPU_RASR_TEX_SHIFT) /* Device           */
#    define MPU_RASR_TEX_BB(bb) ((4|(bb)) << MPU_RASR_TEX_SHIFT)
#      define MPU_RASR_CP_NC    (0)                       /* Non-cacheable */
#      define MPU_RASR_CP_WBRA  (1)                       /* Write back, write and Read- Allocate */
#      define MPU_RASR_CP_WT    (2)                       /* Write through, no Write-Allocate */
#      define MPU_RASR_CP_WB    (4)                       /* Write back, no Write-Allocate */
#  define MPU_RASR_AP_SHIFT     (24)                      /* Bits 24-26: Access permission */
#  define MPU_RASR_AP_MASK      (7 << MPU_RASR_AP_SHIFT)
#    define MPU_RASR_AP_NONO    (0 << MPU_RASR_AP_SHIFT)  /* P:None U:None */
#    define MPU_RASR_AP_RWNO    (1 << MPU_RASR_AP_SHIFT)  /* P:RW   U:None */
#    define MPU_RASR_AP_RWRO    (2 << MPU_RASR_AP_SHIFT)  /* P:RW   U:RO   */
#    define MPU_RASR_AP_RWRW    (3 << MPU_RASR_AP_SHIFT)  /* P:RW   U:RW   */
#    define MPU_RASR_AP_RONO    (5 << MPU_RASR_AP_SHIFT)  /* P:RO   U:None */
#    define MPU_RASR_AP_RORO    (6 << MPU_RASR_AP_SHIFT)  /* P:RO   U:RO   */
#  define MPU_RASR_XN           (1 << 28)                 /* Bit 28: Instruction access disable */

#ifdef CONFIG_ARM_MPU

/*********************************************************************************************
 * Public Function Prototypes
 *********************************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*********************************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *  Allocate the next region
 *
 *********************************************************************************************/

unsigned int mpu_allocregion(void);

/*********************************************************************************************
 * Name: mpu_log2regionceil
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
 *
 *********************************************************************************************/

uint8_t mpu_log2regionceil(size_t size);

/*********************************************************************************************
 * Name: mpu_log2regionfloor
 *
 * Description:
 *   Determine the largest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size >= (1 << l2size)
 *
 *********************************************************************************************/

uint8_t mpu_log2regionfloor(size_t size);

/*********************************************************************************************
 * Name: mpu_subregion
 *
 * Description:
 *   Given (1) the offset to the beginning of valid data, (2) the size of the
 *   memory to be mapped and (2) the log2 size of the mapping to use,
 *   determine the minimal sub-region set to span that memory region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 *********************************************************************************************/

uint32_t mpu_subregion(uintptr_t base, size_t size, uint8_t l2size);

/*********************************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 *********************************************************************************************/

void mpu_control(bool enable, bool hfnmiena, bool privdefena);

/*********************************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 *********************************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                                        uint32_t flags);

/*********************************************************************************************
 * Inline Functions
 *********************************************************************************************/

/*********************************************************************************************
 * Name: mpu_showtype
 *
 * Description:
 *   Show the characteristics of the MPU
 *
 *********************************************************************************************/

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

/*********************************************************************************************
 * Name: mpu_priv_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 *********************************************************************************************/

#define mpu_priv_stronglyordered(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                                               /* Not Cacheable      */ \
                                               /* Not Bufferable     */ \
                           MPU_RASR_S        | /* Shareable          */ \
                           MPU_RASR_AP_RWNO    /* P:RW   U:None      */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_user_flash
 *
 * Description:
 *   Configure a region for user program flash
 *
 *********************************************************************************************/

#define mpu_user_flash(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                                               /* Not Bufferable     */ \
                                               /* Not Shareable      */ \
                           MPU_RASR_AP_RORO    /* P:RO   U:RO        */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_priv_flash
 *
 * Description:
 *   Configure a region for privileged program flash
 *
 *********************************************************************************************/

#define mpu_priv_flash(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                                               /* Not Bufferable     */ \
                                               /* Not Shareable      */ \
                           MPU_RASR_AP_RONO    /* P:RO   U:None      */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_user_intsram
 *
 * Description:
 *   Configure a region as user internal SRAM
 *
 *********************************************************************************************/

#define mpu_user_intsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                                               /* Not Bufferable     */ \
                           MPU_RASR_S        | /* Shareable          */ \
                           MPU_RASR_AP_RWRW    /* P:RW   U:RW        */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_priv_intsram
 *
 * Description:
 *   Configure a region as privileged internal SRAM
 *
 *********************************************************************************************/

#define mpu_priv_intsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size,\
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                                               /* Not Bufferable     */ \
                           MPU_RASR_S        | /* Shareable          */ \
                           MPU_RASR_AP_RWNO    /* P:RW   U:None      */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_user_extsram
 *
 * Description:
 *   Configure a region as user external SRAM
 *
 *********************************************************************************************/

#define mpu_user_extsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                           MPU_RASR_B        | /* Bufferable         */ \
                           MPU_RASR_S        | /* Shareable          */ \
                           MPU_RASR_AP_RWRW    /* P:RW   U:RW        */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_priv_extsram
 *
 * Description:
 *   Configure a region as privileged external SRAM
 *
 *********************************************************************************************/

#define mpu_priv_extsram(base, size) \
  do \
    { \
      /* The configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_SO   | /* Ordered            */ \
                           MPU_RASR_C        | /* Cacheable          */ \
                           MPU_RASR_B        | /* Bufferable         */ \
                           MPU_RASR_S        | /* Shareable          */ \
                           MPU_RASR_AP_RWNO    /* P:RW   U:None      */ \
                                               /* Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged peripheral address space
 *
 *********************************************************************************************/

#define mpu_peripheral(base, size) \
  do \
    { \
      /* Then configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_DEV  | /* Device                */ \
                                               /* Not Cacheable         */ \
                           MPU_RASR_B        | /* Bufferable            */ \
                           MPU_RASR_S        | /* Shareable             */ \
                           MPU_RASR_AP_RWNO  | /* P:RW   U:None         */ \
                           MPU_RASR_XN         /* No Instruction access */); \
    } while (0)

/*********************************************************************************************
 * Name: mpu_user_peripheral
 *
 * Description:
 *   Configure a region as user peripheral address space
 *
 *********************************************************************************************/

#define mpu_user_peripheral(base, size) \
  do \
    { \
      /* Then configure the region */ \
      mpu_configure_region(base, size, \
                           MPU_RASR_TEX_DEV  | /* Device                */ \
                                               /* Not Cacheable         */ \
                           MPU_RASR_B        | /* Bufferable            */ \
                           MPU_RASR_S        | /* Shareable             */ \
                           MPU_RASR_AP_RWRW  | /* P:RW     U:RW         */ \
                           MPU_RASR_XN         /* No Instruction access */); \
    } while (0)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARM_MPU */
#endif /* __ARCH_ARM_SRC_ARMV7M_MPU_H */
