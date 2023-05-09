/****************************************************************************
 * arch/arm/include/imxrt/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_IMXRT_CHIP_H
#define __ARCH_ARM_INCLUDE_IMXRT_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

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
      defined(CONFIG_ARCH_CHIP_MIMXRT1062CVL5A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1064DVL6A) || \
      defined(CONFIG_ARCH_CHIP_MIMXRT1064CVL5A)
/* MIMXRT1061CVL5A - Industrial, Reduced Features, 528MHz
 * MIMXRT1061DVL6A - Consumer, Reduced Features, 600MHz
 * MIMXRT1062CVL5A - Industrial, Full Feature, 528MHz
 * MIMXRT1062DVL6A - Consumer, Full Feature, 600MHz
 * MIMXRT1064CVL5A - Industrial, Full Feature, 528MHz
 * MIMXRT1064DVL6A - Consumer, Full Feature, 600MHz
 */

#  define IMXRT_OCRAM_SIZE            (1024 * 1024) /* 1024Kb OCRAM */
#  define IMXRT_GPIO_NPORTS            9            /* Nine total ports */

#elif defined(CONFIG_ARCH_CHIP_MIMXRT1176DVMAA)
/* MIMXRT1170DVMA TODO
 */
#  define IMXRT_GPIO_NPORTS            13           /* Thirteen total ports */
#else
#  error "Unknown i.MX RT chip type"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds an 8-bit priority value, 0-15. The lower the
 * value, the greater the priority of the corresponding interrupt.  The i.MX
 * RT processor implements only bits[7:4] of each field, bits[3:0] read as
 * zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is min pri */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x40 /* Two bits of interrupt pri used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_IMXRT_CHIP_H */
