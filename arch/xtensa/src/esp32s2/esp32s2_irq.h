/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_irq.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_IRQ_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_IRQ_H

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

#define ESP32S2_CPUINT_LEVEL   ESP32S2_CPUINT_FLAG_LEVEL
#define ESP32S2_CPUINT_EDGE    ESP32S2_CPUINT_FLAG_EDGE

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s2_cpuint_initialize
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

int esp32s2_cpuint_initialize(void);

/****************************************************************************
 * Name:  esp32s2_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
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

int esp32s2_setup_irq(int periphid, int priority, int flags);

/****************************************************************************
 * Name:  esp32s2_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32s2_setup_irq.
 *   It detaches a peripheral interrupt from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   periphid - The peripheral number from irq.h to be detached from the
 *              CPU interrupt.
 *   cpuint   - The CPU interrupt from which the peripheral interrupt will
 *              be detached.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_teardown_irq(int periphid, int cpuint);

/****************************************************************************
 * Name:  esp32s2_irq_noniram_disable
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

void esp32s2_irq_noniram_disable(void);

/****************************************************************************
 * Name:  esp32s2_irq_noniram_enable
 *
 * Description:
 *   Re-enable interrupts disabled by esp32s2_irq_noniram_disable
 *
 * Input Parameters:
 *   None
 *
 * Input Parameters:
 *   None
 *
 ****************************************************************************/

void esp32s2_irq_noniram_enable(void);

/****************************************************************************
 * Name:  esp32s2_irq_noniram_status
 *
 * Description:
 *   Get the current status of non-IRAM interrupts.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if non-IRAM interrupts are enabled, false otherwise.
 *
 ****************************************************************************/

bool esp32s2_irq_noniram_status(void);

/****************************************************************************
 * Name:  esp32s2_irq_set_iram_isr
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

int esp32s2_irq_set_iram_isr(int irq);

/****************************************************************************
 * Name:  esp32s2_irq_unset_iram_isr
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

int esp32s2_irq_unset_iram_isr(int irq);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_IRQ_H */
