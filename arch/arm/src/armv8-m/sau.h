/****************************************************************************
 * arch/arm/src/armv8-m/sau.h
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_SAU_H
#define __ARCH_ARM_SRC_ARMV8_M_SAU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <debug.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SAU Register Bases */

#define SAU_BASE                0xe000edd0

/* SAU Register Offsets */

#define SAU_CTRL_OFFSET         0x0000 /* SAU Control Register */
#define SAU_TYPE_OFFSET         0x0004 /* SAU Type Register */
#define SAU_RNR_OFFSET          0x0008 /* SAU Region Number Register */
#define SAU_RBAR_OFFSET         0x000c /* SAU Region Base Address Register */
#define SAU_RLAR_OFFSET         0x0010 /* SAU Region Limit Address Register */
#define SAU_SFSR_OFFSET         0x0014 /* Secure Fault Status Register */
#define SAU_SFAR_OFFSET         0x0018 /* Secure Fault Address Register */

/* SAU Register Addresses */

#define SAU_CTRL                (SAU_BASE + SAU_CTRL_OFFSET)
#define SAU_TYPE                (SAU_BASE + SAU_TYPE_OFFSET)
#define SAU_RNR                 (SAU_BASE + SAU_RNR_OFFSET)
#define SAU_RBAR                (SAU_BASE + SAU_RBAR_OFFSET)
#define SAU_RLAR                (SAU_BASE + SAU_RLAR_OFFSET)
#define SAU_SFSR                (SAU_BASE + SAU_SFSR_OFFSET)
#define SAU_SFAR                (SAU_BASE + SAU_SFAR_OFFSET)

/* SAU Control Register Bit Definitions */

#define SAU_CTRL_ENABLE         (1 << 0)   /* Bit 0: Enable the SAU */
#define SAU_CTRL_ALLNS          (1 << 1)   /* Bit 1: All Non-secure */

/* SAU Type Register Bit Definitions */

#define SAU_TYPE_SREGION_SHIFT  (0)        /* Bits 0-7: Number SAU regions */
#define SAU_TYPE_SREGION_MASK   (0xff << SAU_TYPE_SREGION_SHIFT)

/* SAU Region Base Address Register Bit Definitions */

#define SAU_RBAR_BASE_MASK      0xffffffe0 /* Bits 5-31: Region base addrese */

/* SAU Region Region Limit Address Register Bit Definitions */

#define SAU_RLAR_ENABLE         (1 << 0)   /* Bit 0: Region enable */
#define SAU_RLAR_NSC            (1 << 1)   /* Bit 1: Non-secure callable */
#define SAU_RLAR_LIMIT_MASK     0xffffffe0 /* Bits 5-31: Region limit address */

/* Secure Fault Status Register Definitions */

#define SAU_SFSR_MASK           (0xf)      /* Secure Fault Status Register Mask */

#define SAU_SFSR_INVEP          (1 << 0)   /* Bit 0: INVEP Mask */
#define SAU_SFSR_INVIS          (1 << 1)   /* Bit 1: INVIS Mask */
#define SAU_SFSR_INVER          (1 << 2)   /* Bit 2: INVER Mask */
#define SAU_SFSR_AUVIOL         (1 << 3)   /* Bit 3: AUVIOL Mask */
#define SAU_SFSR_INVTRAN        (1 << 4)   /* Bit 4: INVTRAN Mask */
#define SAU_SFSR_LSPERR         (1 << 5)   /* Bit 5: LSPERR Mask */
#define SAU_SFSR_SFARVALID      (1 << 6)   /* Bit 6: SFARVALID Mask */
#define SAU_SFSR_LSERR          (1 << 7)   /* Bit 7: LSERR Mask */

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
 * Name: sau_control
 *
 * Description:
 *   Configure and enable (or disable) the SAU
 *
 ****************************************************************************/

void sau_control(bool enable, bool allns);

/****************************************************************************
 * Name: sau_configure_region
 *
 * Description:
 *   Configure a region for secure attribute
 *
 ****************************************************************************/

void sau_configure_region(uintptr_t base, size_t size, uint32_t flags);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sau_showtype
 *
 * Description:
 *   Show the characteristics of the SAU
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SCHED_INFO
#  define sau_showtype() \
    do \
      { \
        uint32_t regval = getreg32(SAU_TYPE); \
        sinfo("SAU Regions: %d\n", \
          (regval & SAU_TYPE_SREGION_MASK) >> SAU_TYPE_SREGION_SHIFT); \
    } while (0)
#else
#  define sau_showtype() do { } while (0)
#endif

/****************************************************************************
 * Name: sau_non_secure
 *
 * Description:
 *   Configure a non-secure region
 *
 ****************************************************************************/

#define sau_non_secure(base, size) \
  do \
    { \
      /* The configure the region */ \
      sau_configure_region(base, size, 0); \
    } while (0)

/****************************************************************************
 * Name: sau_non_secure_callable
 *
 * Description:
 *   Configure a non-secure callable region
 *
 ****************************************************************************/

#define sau_non_secure_callable(base, size) \
  do \
    { \
      /* The configure the region */ \
      sau_configure_region(base, size, SAU_RLAR_NSC); \
    } while (0)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARMV8_M_SAU_H */
