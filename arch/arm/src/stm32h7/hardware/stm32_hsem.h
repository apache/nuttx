/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_hsem.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_HSEM_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_HSEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_HSEM_CHANS             (32)
#define HSEM_COREID_CPU1             (3)
#define HSEM_COREID_CPU2             (1)

/* Register Offsets *********************************************************/

#define STM32_HSEM_RX_OFFSET(n)       (0x0000 + (0x4 * n))   /* HSEM register semaphore x */
#define STM32_HSEM_RLRX_OFFSET(n)     (0x0080 + (0x4 * n))   /* HSEM read lock register semaphore x */
#define STM32_HSEM_CXIER_OFFSET(n)    (0x0100 + (0x10 * n))  /* HSEM interrupt enable register */
#define STM32_HSEM_CXICR_OFFSET(n)    (0x0104 + (0x10 * n))  /* HSEM interrupt clear register */
#define STM32_HSEM_CXISR_OFFSET(n)    (0x0108 + (0x10 * n))  /* HSEM interrupt status register */
#define STM32_HSEM_CXMISR_OFFSET(n)   (0x010c + (0x10 * n))  /* HSEM interrupt status register */
#define STM32_HSEM_CR_OFFSET          (0x0140)               /* HSEM clear register */
#define STM32_HSEM_KEYR_OFFSET        (0x0144)               /* HSEM semaphore clear key */

/* Register Addresses *******************************************************/

#define STM32_HSEM_RX(x)              (STM32_HSEM_BASE+STM32_HSEM_RX_OFFSET(x))
#define STM32_HSEM_RLRX(x)            (STM32_HSEM_BASE+STM32_HSEM_RLRX_OFFSET(x))
#define STM32_HSEM_CXIER(x)           (STM32_HSEM_BASE+STM32_HSEM_CXIER_OFFSET(x))
#define STM32_HSEM_CXICR(x)           (STM32_HSEM_BASE+STM32_HSEM_CXICR_OFFSET(x))
#define STM32_HSEM_CXISR(x)           (STM32_HSEM_BASE+STM32_HSEM_CXISR_OFFSET(x))
#define STM32_HSEM_CXMISR(x)          (STM32_HSEM_BASE+STM32_HSEM_CXMISR_OFFSET(x))
#define STM32_HSEM_CXMISRCR           (STM32_HSEM_BASE+STM32_HSEM_CR_OFFSET)
#define STM32_HSEM_KEYR               (STM32_HSEM_BASE+STM32_HSEM_KEYR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Semaphore x registers */

#define HSEM_SEMX_PROCID_SHIFT      (0)        /* Bits 0-8: Semaphore PROCID */
#define HSEM_SEMX_PROCID_MASK       (0xff << HSEM_SEMX_PROCID_SHIFT)
#define HSEM_SEMX_COREID_SHIFT      (8)        /* Bits 8-11: Semaphore COREID */
#define HSEM_SEMX_COREID_MASK       (0xf << HSEM_SEMX_COREID_SHIFT)
                                               /* Bits 12-30: reserved */
#define HSEM_SEMX_LOCK              (1 << 31)  /* Bit 31: Lock indication */

/* Clear register */

#define HSEM_CR_COREID_SHIFT        (8) /* Bits 8-11: COREID of semaphores to be cleared */
#define HSEM_CR_COREID_MASK         (0xf << HSEM_CR_COREID_SHIFT)
#define HSEM_CR_KEY_SHIFT           (16) /* Bits 16-31: Semaphore clear Key */
#define HSEM_CR_KEY_MASK            (0xffff << HSEM_CR_KEY_SHIFT)

/* Interrupts */

#define HSEM_CHAN_ID(n)             (1 << n)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_HSEM_H */
