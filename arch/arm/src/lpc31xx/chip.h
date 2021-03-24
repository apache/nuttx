/****************************************************************************
 * arch/arm/src/lpc31xx/chip.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_CHIP_H
#define __ARCH_ARM_SRC_LPC31XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_LPC3130)
#  undef  HAVE_INTSRAM1                  /* 96Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3131)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3152)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3154)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  define HAVE_AESENGINE 1               /* AES engine */
#else
#  error "Unsupported LPC31XX architecture"
#  undef  HAVE_INTSRAM1                  /* No INTSRAM1 */
#  define LPC31_NDMACH   0               /* No DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#endif

/* Cache line sizes (in bytes) */

#define ARM_DCACHE_LINESIZE 32           /* 32 bytes (8 words) */
#define ARM_ICACHE_LINESIZE 32           /* 32 bytes (8 words) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_CHIP_H */
