/*****************************************************************************
 * arch/arm/include/imxrt/chip.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_IMXRT_CHIP_H
#define __ARCH_ARM_INCLUDE_IMXRT_CHIP_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_MIMXRT1021CAG4A) || \
    defined(CONFIG_ARCH_CHIP_MIMXRT1021CAF4A) || \
    defined(CONFIG_ARCH_CHIP_MIMXRT1021DAF5A) || \
    defined(CONFIG_ARCH_CHIP_MIMXRT1021DAG5A)

/*  MIMXRT1021CAG4A - 144 pin, 400MHz Industrial
 *  MIMXRT1021CAF4A - 100 pin, 400MHz Industrial
 *  MIMXRT1021DAF5A - 100 pin, 500MHz Consumer
 *  MIMXRT1021DAG5A - 144 pin, 500MHz Consumer
 */

#  define IMXRT_OCRAM_SIZE      (256 * 1024) /* 256Kb OCRAM */
#  define IMXRT_GPIO_NPORTS     5            /* Five total ports */
                                             /* but 4 doesn't exist */

#elif defined(CONFIG_ARCH_CHIP_MIMXRT1051DVL6A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1051CVL5A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1052DVL6A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1052CVL5A)
/* MIMXRT1051CVL5A - Industrial, Reduced Features, 528MHz
 * MIMXRT1051DVL6A - Consumer, Reduced Features, 600MHz
 * MIMXRT1052CVL5A - Industrial, Full Feature, 528MHz
 * MIMXRT1052DVL6A - Consumer, Full Feature, 600MHz
 */

#  define IMXRT_OCRAM_SIZE            (512 * 1024) /* 512Kb OCRAM */
#  define IMXRT_GPIO_NPORTS            5           /* Five total ports */

#elif defined(CONFIG_ARCH_CHIP_MIMXRT1061DVL6A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1061CVL5A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1062DVL6A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1062CVL5A)
/* MIMXRT1061CVL5A - Industrial, Reduced Features, 528MHz
 * MIMXRT1061DVL6A - Consumer, Reduced Features, 600MHz
 * MIMXRT1062CVL5A - Industrial, Full Feature, 528MHz
 * MIMXRT1062DVL6A - Consumer, Full Feature, 600MHz
 */

#  define IMXRT_OCRAM_SIZE            (1024 * 1024) /* 1024Kb OCRAM */
#  define IMXRT_GPIO_NPORTS            9            /* Nine total ports */
#else
#  error "Unknown i.MX RT chip type"
#endif

/* NVIC priority levels ******************************************************
/* Each priority field holds an 8-bit priority value, 0-15. The lower the
 * value, the greater the priority of the corresponding interrupt.  The i.MX
 * RT processor implements only bits[7:4] of each field, bits[3:0] read as
 * zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is min pri */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x40 /* Two bits of interrupt pri used */

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/*****************************************************************************
 * Public Data
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_IMXRT_CHIP_H */
