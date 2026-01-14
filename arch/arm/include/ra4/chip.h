/****************************************************************************
 * arch/arm/include/ra4/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RA_CHIP_H
#define __ARCH_ARM_INCLUDE_RA_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

/* RA Family */

/* Common FEATURES
 * Flash        256KB
 * DataFlash    8KB
 * SRAM         32+16KB
 */

/* Internal memory */

#  define RA_FLASH_SIZE            (256*1024)   /* 256KB */
#  define RA_SRAM0_SIZE            (48*1024)    /* 48KB */

/* FEATURE      R7FA4M1ABxxFP  R7FA4M1ABxxLJ  R7FA4M1ABxxFM R7FA4M1ABxxNB
 * -----------  -------------  -------------  ------------- -------------
 * Package      LQFP100        LGA100         LQFP64        QFNP64
 * No. PIOs     81             81             49            49
 * SCI          4              4              4             4
 *
 * FEATURE      R7FA4M1ABxxFL R7FA4M1ABxxNE R7FA4M1ABxxNF
 * -----------  ------------- ------------- -------------
 * Package      LQFP48        QFNP48        QFN40
 * No. PIOs     33            33            25
 * SCI          4             4             4
 */

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-15. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:4] of each field, bits[3:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x10 /* Four bits of interrupt priority used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_RA_CHIP_H */
