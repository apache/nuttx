/****************************************************************************
 * arch/xtensa/src/common/mpu.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_MPU_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/core-isa.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#  include <assert.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPU_VADDR_MASK ~(XCHAL_MPU_ALIGN - (1))

#define MPU_ENTRY_AS(vaddr, valid) \
        (((vaddr) & MPU_VADDR_MASK) | \
        ((valid) & (0x1)))

#define MPU_ENTRY_AR(access, memtype) \
        (((ENCODE_MEMORY_TYPE(memtype)) << (12)) | \
        (((access) & (0xf)) << (8)))

/****************************************************************************
 *  MPU access rights constants
 ****************************************************************************/

#define MPU_AR_NONE                  (0)    /* no access */
#define MPU_AR_R                     (4)    /* Kernel read, User no access*/
#define MPU_AR_RX                    (5)    /* Kernel read/execute, User no access */
#define MPU_AR_RW                    (6)    /* Kernel read/write, User no access */
#define MPU_AR_RWX                   (7)    /* Kernel read/write/execute, User no access */
#define MPU_AR_Ww                    (8)    /* Kernel write, User write */
#define MPU_AR_RWrwx                 (9)    /* Kernel read/write , User read/write/execute */
#define MPU_AR_RWr                   (10)   /* Kernel read/write, User read */
#define MPU_AR_RWXrx                 (11)   /* Kernel read/write/execute, User read/execute */
#define MPU_AR_Rr                    (12)   /* Kernel read, User read */
#define MPU_AR_RXrx                  (13)   /* Kernel read/execute, User read/execute */
#define MPU_AR_RWrw                  (14)   /* Kernel read/write, User read/write */
#define MPU_AR_RWXrwx                (15)   /* Kernel read/write/execute, User read/write/execute */
#define MPU_AR_WIDTH                 4      /* Bits used to encode access rights */

/****************************************************************************
 * MPU access memtype constants
 ****************************************************************************/

#define MPU_MEM_DEVICE               (0x00008000)
#define MPU_MEM_NON_CACHEABLE        (0x00090000)
#define MPU_MEM_WRITETHRU_NOALLOC    (0x00080000)
#define MPU_MEM_WRITETHRU            (0x00040000)
#define MPU_MEM_WRITETHRU_WRITEALLOC (0x00060000)
#define MPU_MEM_WRITEBACK_NOALLOC    (0x00050000)
#define MPU_MEM_WRITEBACK            (0x00070000)
#define MPU_MEM_INTERRUPTIBLE        (0x08000000)
#define MPU_MEM_BUFFERABLE           (0x01000000)
#define MPU_MEM_NON_SHAREABLE        (0x00000000)
#define MPU_MEM_INNER_SHAREABLE      (0x02000000)
#define MPU_MEM_OUTER_SHAREABLE      (0x04000000)
#define MPU_MEM_SYSTEM_SHAREABLE     (0x06000000)

/****************************************************************************
 * Layout of the MPU memory type specifier for: ENCODE_MEMORY_TYPE()
 *
 * Bits 0-3  - reserved for pass through of accessRights
 * Bits 4-12 - reserved for pass through of memoryType bits
 * Bit  13   - indicates to use existing access rights of region
 * Bit  14   - indicates to use existing memory type of region
 * Bit  15   - indicates device
 * Bit  16-19- system cache properties
 * Bit  20-23- local cache properties
 * Bit  24   - indicates bufferable
 * Bit  25-26- encodes shareability (1=inner, 2=outer, 3=system)
 * Bit  27   - indicates interruptible
 * Bits 28-31- reserved for future use
 ****************************************************************************/

#define SYSTEM_CACHE_BITS            (0x000f0000)
#define LOCAL_CACHE_BITS             (0x00f00000)
#define SYSTEM_RWC_MASK              (0x00070000)
#define LOCAL_RWC_MASK               (0x00700000)
#define SHIFT_RWC                    16

#define MEM_ANY_SHAREABLE(x) (((x & MPU_MEM_SYSTEM_SHAREABLE) \
        != (0)) ? (1) : (0))

#define MEM_INNER_SHAREABLE(x) (((x & MPU_MEM_SYSTEM_SHAREABLE) \
        == MPU_MEM_INNER_SHAREABLE) ? (1) : (0))

#define MEM_IS_BUFFERABLE(x) (((x & MPU_MEM_BUFFERABLE) != (0)) ? \
        (1) : (0))

#define MEM_IS_DEVICE(x) (((x & MPU_MEM_DEVICE) != (0)) ? (1) : (0))

#define NON_CACHEABLE_DOMAIN(x) \
        ((MEM_IS_DEVICE(x) != (0)) || \
        (MEM_ANY_SHAREABLE(x) != (0)) ? (0x3) : (0))

#define CACHEABLE_DOMAIN(x)  ((MEM_ANY_SHAREABLE(x) != (0)) ? \
        (0x3) : (0x1))

#define MEM_CACHE_MASK(x) (x & SYSTEM_CACHE_BITS)

#define IS_SYSTEM_NONCACHEABLE(x) \
        (((MEM_CACHE_MASK(x) & MPU_MEM_NON_CACHEABLE) == \
                MPU_MEM_NON_CACHEABLE) ? (1) : (0))

#define ENCODE_DEVICE(x) \
        (((((x & MPU_MEM_INTERRUPTIBLE) != (0)) \
        ? (1) : (0)) << 3) | \
        ((NON_CACHEABLE_DOMAIN(x) << 1) | MEM_IS_BUFFERABLE(x)))

#define ENCODE_SYSTEM_NONCACHEABLE(x) \
        (((x & LOCAL_CACHE_BITS) != (0)) && \
        (((x & LOCAL_CACHE_BITS) >> 4) != MPU_MEM_NON_CACHEABLE)) ? \
        ((0x89) | (((x) & LOCAL_CACHE_BITS) >> SHIFT_RWC)) :\
        ((((0x18) | (NON_CACHEABLE_DOMAIN(x) << 1)) \
                | MEM_IS_BUFFERABLE(x)))

#define ENCODE_SYSTEM_CACHEABLE(x) \
        (((((x & LOCAL_CACHE_BITS) >> 4) & MPU_MEM_NON_CACHEABLE) == \
               MPU_MEM_NON_CACHEABLE) ? \
        (CACHEABLE_DOMAIN(x) << 4) : \
        ENCODE_SYSTEM_CACHEABLE_LOCAL_CACHEABLE(x)) | \
        ((MEM_INNER_SHAREABLE(x) << 3) | \
                 ((MEM_CACHE_MASK(x) & SYSTEM_RWC_MASK) \
                 >> SHIFT_RWC))

#define ENCODE_SYSTEM_CACHEABLE_LOCAL_CACHEABLE(x) \
        ((CACHEABLE_DOMAIN(x) << 7) | (((((x & LOCAL_CACHE_BITS) != (0)) ? \
                (x & LOCAL_CACHE_BITS) : \
                ((MEM_CACHE_MASK(x) << 4)) \
        & (LOCAL_RWC_MASK)) >> SHIFT_RWC)))

#define ENCODE_MEMORY_TYPE(x) \
        (((x & (0xffffe000)) != (0)) ? \
        ((MEM_IS_DEVICE((x)) != (0)) ? ENCODE_DEVICE((x)) : \
        ((IS_SYSTEM_NONCACHEABLE((x)) != (0)) ? \
        ENCODE_SYSTEM_NONCACHEABLE((x)) : \
        ENCODE_SYSTEM_CACHEABLE((x)))) : x)

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
 * Name: mpu_allocregion
 *
 * Description:
 *  Allocate the next region
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void);

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 ****************************************************************************/

void mpu_control(bool enable);

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                          uint32_t acc, uint32_t memtype);

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
                          MPU_AR_RWX, \
                          MPU_MEM_DEVICE); \
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
                          MPU_AR_RXrx, \
                          MPU_MEM_WRITEBACK);\
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
      /* The configure the region */   \
      mpu_configure_region(base, size, \
                          MPU_AR_RX, \
                          MPU_MEM_WRITEBACK);\
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
                          MPU_AR_RWXrwx, \
                          MPU_MEM_WRITEBACK);\
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
      mpu_configure_region(base, size,\
                          MPU_AR_RWX, \
                          MPU_MEM_WRITEBACK);\
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
                          MPU_AR_RWXrwx, \
                          MPU_MEM_WRITEBACK);\
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
      mpu_configure_region(base, size,  \
                          MPU_AR_RWX, \
                          MPU_MEM_WRITEBACK);\
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
      /* Then configure the region */  \
      mpu_configure_region(base, size, \
                          MPU_AR_RW, \
                          MPU_MEM_DEVICE);\
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
      /* Then configure the region */     \
      mpu_configure_region(base, size,    \
                          MPU_AR_RWrw,  \
                          MPU_MEM_DEVICE);\
    } while (0)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_MPU_H */
