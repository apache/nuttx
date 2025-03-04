/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_IRQ_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
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

/* CPU interrupt flags.
 * esp_setup_irq() will check bit 1 for IRAM requirement and
 * bit 0 for trigger type.
 *
 * OR'ed values of irq_trigger_e and irq_iram_e define interrupt type.
 *      | IRAM | TRIGGER |
 * Bits |   1  |    0    |
 */

/* CPU interrupt trigger types */

typedef enum irq_trigger_e
{
  ESP_IRQ_TRIGGER_LEVEL    = 0, /* Level-triggered interrupts */
  ESP_IRQ_TRIGGER_EDGE     = 1, /* Edge-triggered interrupts */
} irq_trigger_t;

/* CPU interrupt IRAM enabled */

typedef enum irq_iram_e
{
  ESP_IRQ_NON_IRAM    = (0 << 1), /* Non-IRAM interrupt */
  ESP_IRQ_IRAM        = (1 << 1), /* IRAM interrupt */
} irq_iram_t;

/* CPU interrupt priority levels */

typedef enum irq_priority_e
{
  ESP_IRQ_PRIORITY_1       = 1,                 /* Priority Level 1 */
  ESP_IRQ_PRIORITY_2       = 2,                 /* Priority Level 2 */
  ESP_IRQ_PRIORITY_3       = 3,                 /* Priority Level 3 */
  ESP_IRQ_PRIORITY_4       = 4,                 /* Priority Level 4 */
  ESP_IRQ_PRIORITY_5       = 5,                 /* Priority Level 5 */
  ESP_IRQ_PRIORITY_6       = 6,                 /* Priority Level 6 */
  ESP_IRQ_PRIORITY_7       = 7,                 /* Priority Level 7 */
  ESP_IRQ_PRIORITY_8       = 8,                 /* Priority Level 8 */
  ESP_IRQ_PRIORITY_9       = 9,                 /* Priority Level 9 */
  ESP_IRQ_PRIORITY_10      = 10,                /* Priority Level 10 */
  ESP_IRQ_PRIORITY_11      = 11,                /* Priority Level 11 */
  ESP_IRQ_PRIORITY_12      = 12,                /* Priority Level 12 */
  ESP_IRQ_PRIORITY_13      = 13,                /* Priority Level 13 */
  ESP_IRQ_PRIORITY_14      = 14,                /* Priority Level 14 */
  ESP_IRQ_PRIORITY_15      = 15,                /* Priority Level 15 */
  ESP_IRQ_PRIORITY_DEFAULT = ESP_IRQ_PRIORITY_1 /* Default Priority */
} irq_priority_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_route_intr
 *
 * Description:
 *   Assign an interrupt source to a pre-allocated CPU interrupt.
 *
 * Input Parameters:
 *   source        - Interrupt source (see irq.h) to be assigned to a CPU
 *                   interrupt.
 *   cpuint        - Pre-allocated CPU interrupt to which the interrupt
 *                   source will be assigned.
 *   priority      - Interrupt priority.
 *   type          - Interrupt trigger type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_route_intr(int source, int cpuint, irq_priority_t priority,
                    irq_trigger_t type);

/****************************************************************************
 * Name: esp_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to a given interrupt source.
 *
 * Input Parameters:
 *   source        - The interrupt source from irq.h to be assigned to
 *                   a CPU interrupt.
 *   priority      - Interrupt priority.
 *   type          - Interrupt trigger type.
 *
 * Returned Value:
 *   Allocated CPU interrupt.
 *
 ****************************************************************************/

int esp_setup_irq(int source, irq_priority_t priority, int type);

/****************************************************************************
 * Name: esp_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp_setup_irq.
 *   It detaches an interrupt source from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   source        - The interrupt source from irq.h to be detached from the
 *                   CPU interrupt.
 *   cpuint        - The CPU interrupt from which the interrupt source will
 *                   be detached.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_teardown_irq(int source, int cpuint);

/****************************************************************************
 * Name: esp_intr_noniram_disable
 *
 * Description:
 *   Disable interrupts that aren't specifically marked as running from IRAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_intr_noniram_disable(void);

/****************************************************************************
 * Name: esp_intr_noniram_enable
 *
 * Description:
 *   Enable interrupts that aren't specifically marked as running from IRAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_intr_noniram_enable(void);

/****************************************************************************
 * Name:  esp_get_irq
 *
 * Description:
 *   This function returns the IRQ associated with a CPU interrupt
 *
 * Input Parameters:
 *   cpuint - The CPU interrupt associated to the IRQ
 *
 * Returned Value:
 *   The IRQ associated with such CPU interrupt or CPUINT_UNASSIGNED if
 *   IRQ is not yet assigned to a CPU interrupt.
 *
 ****************************************************************************/

int esp_get_irq(int cpuint);

/****************************************************************************
 * Name:  esp_set_irq
 *
 * Description:
 *   This function assigns a CPU interrupt to a specific IRQ number. It
 *   updates the mapping between IRQ numbers and CPU interrupts, allowing
 *   the system to correctly route hardware interrupts to the appropriate
 *   handlers. Please note that this function is intended to be used only
 *   when a CPU interrupt is already assigned to an IRQ number. Otherwise,
 *   please check esp_setup_irq.
 *
 * Input Parameters:
 *   irq    - The IRQ number to be associated with the CPU interrupt.
 *   cpuint - The CPU interrupt to be associated with the IRQ number.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_set_irq(int irq, int cpuint);

/****************************************************************************
 * Name:  esp_irq_set_iram_isr
 *
 * Description:
 *   Set the ISR associated to an IRQ as a IRAM-enabled ISR.
 *
 * Input Parameters:
 *   irq - The associated IRQ to set
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_irq_set_iram_isr(int irq);

/****************************************************************************
 * Name:  esp32s3_irq_unset_iram_isr
 *
 * Description:
 *   Set the ISR associated to an IRQ as a non-IRAM ISR.
 *
 * Input Parameters:
 *   irq - The associated IRQ to set
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_irq_unset_iram_isr(int irq);

/****************************************************************************
 * Name:  esp_irq_noniram_status
 *
 * Description:
 *   Get the current status of non-IRAM interrupts on a specific CPU core
 *
 * Input Parameters:
 *   cpu - The CPU to check the non-IRAM interrupts state
 *
 * Returned Value:
 *   true if non-IRAM interrupts are enabled, false otherwise.
 *
 ****************************************************************************/

bool esp_irq_noniram_status(int cpu);

/****************************************************************************
 * Name:  esp_get_iram_interrupt_records
 *
 * Description:
 *   This function copies the vector that keeps track of the IRQs that ran
 *   when non-IRAM interrupts were disabled.
 *
 * Input Parameters:
 *
 *   irq_count - A previously allocated pointer to store the counter of the
 *               interrupts that ran when non-IRAM interrupts were disabled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_get_iram_interrupt_records(uint64_t *irq_count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_IRQ_H */
