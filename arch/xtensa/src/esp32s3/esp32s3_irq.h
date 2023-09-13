/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_irq.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

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

/* CPU interrupt types. */

#define ESP32S3_CPUINT_LEVEL   ESP32S3_CPUINT_FLAG_LEVEL
#define ESP32S3_CPUINT_EDGE    ESP32S3_CPUINT_FLAG_EDGE

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s3_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32s3_cpuint_initialize(void);

/****************************************************************************
 * Name:  esp32s3_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   priority - Interrupt's priority (1 - 5).
 *   flags    - An ORred mask of the ESP32S3_CPUINT_FLAG_* defines. These
 *              restrict the choice of interrupts that this routine can
 *              choose from.
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp32s3_setup_irq(int cpu, int periphid, int priority, int flags);

/****************************************************************************
 * Name:  esp32s3_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32s3_setup_irq.
 *   It detaches a peripheral interrupt from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be detached from the
 *              CPU interrupt.
 *   cpuint   - The CPU interrupt from which the peripheral interrupt will
 *              be detached.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_teardown_irq(int cpu, int periphid, int cpuint);

/****************************************************************************
 * Name:  esp32s3_getirq
 *
 * Description:
 *   This function returns the IRQ associated with a CPU interrupt
 *
 * Input Parameters:
 *   cpu    - The CPU core of the IRQ being queried
 *   cpuint - The CPU interrupt associated to the IRQ
 *
 * Returned Value:
 *   The IRQ associated with such CPU interrupt or CPUINT_UNASSIGNED if
 *   IRQ is not yet assigned to a CPU interrupt.
 *
 ****************************************************************************/

int esp32s3_getirq(int cpu, int cpuint);

/****************************************************************************
 * Name:  esp32s3_getcpuint_from_irq
 *
 * Description:
 *   This function returns the CPU interrupt associated with an IRQ
 *
 * Input Parameters:
 *   irq - The IRQ associated with a CPU interrupt
 *   cpu - Pointer to store the CPU core of the CPU interrupt
 *
 * Returned Value:
 *   The CPU interrupt associated with such IRQ or IRQ_UNMAPPED if
 *   CPU interrupt is not mapped to an IRQ.
 *
 ****************************************************************************/

int esp32s3_getcpuint_from_irq(int irq, int *cpu);

/****************************************************************************
 * Name:  esp32s3_irq_noniram_disable
 *
 * Description:
 *   Disable interrupts that aren't specifically marked as running from IRAM
 *
 * Input Parameters:
 *   None
 *
 * Input Parameters:
 *   None
 *
 ****************************************************************************/

void esp32s3_irq_noniram_disable(void);

/****************************************************************************
 * Name:  esp32s3_irq_noniram_enable
 *
 * Description:
 *   Re-enable interrupts disabled by esp32s3_irq_noniram_disable
 *
 * Input Parameters:
 *   None
 *
 * Input Parameters:
 *   None
 *
 ****************************************************************************/

void esp32s3_irq_noniram_enable(void);

/****************************************************************************
 * Name:  esp32s3_irq_noniram_status
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

bool esp32s3_irq_noniram_status(int cpu);

/****************************************************************************
 * Name:  esp32s3_irq_set_iram_isr
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

int esp32s3_irq_set_iram_isr(int irq);

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

int esp32s3_irq_unset_iram_isr(int irq);

#ifdef CONFIG_ESP32S3_IRAM_ISR_DEBUG

/****************************************************************************
 * Name:  esp32s3_get_iram_interrupt_records
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

void esp32s3_get_iram_interrupt_records(uint64_t *irq_count);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H */
