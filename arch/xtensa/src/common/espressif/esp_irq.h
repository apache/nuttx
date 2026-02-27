/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_irq.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_IRQ_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "esp_intr_types.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_UNMAPPED      (intr_handle_t)NULL

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
  ESP_IRQ_PRIORITY_DEFAULT = ESP_IRQ_PRIORITY_1 /* Default Priority */
} irq_priority_t;

/* Adapter from NuttX to Espressif's interrupt handler */

struct intr_adapter_from_nuttx
{
  int (*func)(int irq, void *context, void *arg); /* Interrupt callback function */
  int irq;                                        /* Interrupt number */
  void *context;                                  /* Interrupt context */
  void *arg;                                      /* Interrupt private data */
};

struct intr_adapter_to_nuttx
{
  void (*handler)(void *arg);                    /* Interrupt handler */
  void *arg;                                     /* Interrupt handler argument */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

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
 *   handler       - Interrupt handler.
 *   arg           - Interrupt handler argument.
 *
 * Returned Value:
 *   Allocated CPU interrupt on success, or a negated errno on failure.
 *
 ****************************************************************************/

int esp_setup_irq(int source,
                  irq_priority_t priority,
                  int type,
                  xcpt_t handler,
                  void *arg);

int esp_setup_irq_with_flags(int source,
                             int flags,
                             xcpt_t handler,
                             void *arg);

int esp_setup_irq_intrstatus(int source,
                             irq_priority_t priority,
                             int type,
                             uint32_t intrstatusreg,
                             uint32_t intrstatusmask,
                             xcpt_t handler,
                             void *arg);

int esp_setup_irq_with_flags_intrstatus(int source,
                                        int flags,
                                        uint32_t intrstatusreg,
                                        uint32_t intrstatusmask,
                                        xcpt_t handler,
                                        void *arg);

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
 * Name:  esp_get_cpuint
 *
 * Description:
 *   This function returns the CPU interrupt associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   The CPU interrupt associated with such IRQ or a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp_get_cpuint(int cpu, int irq);

/****************************************************************************
 * Name:  esp_set_handle
 *
 * Description:
 *   This function sets the handle associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *   handle - The handle to be associated with the IRQ
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_set_handle(int cpu, int irq, intr_handle_t handle);

/****************************************************************************
 * Name:  esp_get_handle
 *
 * Description:
 *   This function gets the handle associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   The handle associated with the IRQ or IRQ_UNMAPPED if no handle is
 *   associated with the IRQ.
 *
 ****************************************************************************/

intr_handle_t esp_get_handle(int cpu, int irq);

/****************************************************************************
 * Name: esp_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

void esp_cpuint_initialize(void);

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
 *   cpu -       The CPU to retrieve the interrupt records for
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
void esp_get_iram_interrupt_records(uint64_t *irq_count, int cpu);
#endif

/****************************************************************************
 * Name: esp_dump_cpuint_map
 *
 * Description:
 *   Dump the contents of g_handle_map for debugging purposes.
 *   This function is useful when debugging unexpected interrupt handlers.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dump_cpuint_map(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_IRQ_H */
