/****************************************************************************
 * arch/arm/src/armv8-m/ram_vectors.h
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

#ifndef __ARCH_ARM_SRC_COMMON_ARMV8_M_RAM_VECTORS_H
#define __ARCH_ARM_SRC_COMMON_ARMV8_M_RAM_VECTORS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_RAMVECTORS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the size of the vector table (in 4-byte entries).  This size
 * includes the (1) the peripheral interrupts, (2) space for 15 Cortex-M
 * exceptions, and (3) IDLE stack pointer which lies at the beginning of the
 * table.
 */

#define ARMV8M_VECTAB_SIZE (ARMV8M_PERIPHERAL_INTERRUPTS + 16)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If CONFIG_ARCH_RAMVECTORS is defined, then the ARM logic must provide
 * ARM-specific implementations of irq_initialize(), irq_attach(), and
 * irq_dispatch.  In this case, it is also assumed that the ARM vector
 * table resides in RAM, has the name g_ram_vectors, and has been
 * properly positioned and aligned in memory by the linker script.
 *
 * REVISIT: Can this alignment requirement vary from core-to-core?  Yes, it
 * depends on the number of vectors supported by the MCU. The safest thing
 * to do is to put the vector table at the beginning of RAM in order to force
 * the highest alignment possible.
 */

extern up_vector_t g_ram_vectors[ARMV8M_VECTAB_SIZE]
  __attribute__ ((section (".ram_vectors"), aligned (128)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_ramvec_initialize
 *
 * Description:
 *   Copy vectors to RAM an configure the NVIC to use the RAM vectors.
 *
 ****************************************************************************/

void arm_ramvec_initialize(void);

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   This is the default, common vector handling entrypoint.
 *
 ****************************************************************************/

void exception_common(void);

/****************************************************************************
 * Name: arm_ramvec_attach
 *
 * Description:
 *   Configure the ram vector table so that IRQ number 'irq' will be
 *   dipatched by hardware to 'vector'
 *
 ****************************************************************************/

int arm_ramvec_attach(int irq, up_vector_t vector);

#endif /* CONFIG_ARCH_RAMVECTORS */
#endif /* __ARCH_ARM_SRC_COMMON_ARMV8_M_RAM_VECTORS_H */
