/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_irq.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C6_ESP32C6_IRQ_H
#define __ARCH_RISCV_SRC_ESP32C6_ESP32C6_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

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

#define ESP32C6_INT_LEVEL           (0 << 0)
#define ESP32C6_INT_EDGE            (1 << 0)

#define ESP32C6_INT_PRIO_MIN        1
#define ESP32C6_INT_PRIO_MAX        7

#define ESP32C6_INT_PRIO_DEF        1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c6_bind_irq
 *
 * Description:
 *   Bind IRQ and resource with given parameters.
 *
 * Input Parameters:
 *   cpuint    - CPU interrupt ID
 *   periphid  - Peripheral ID
 *   prio      - Interrupt priority
 *   flags     - Interrupt flags
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_bind_irq(uint8_t cpuint, uint8_t periphid, uint8_t prio,
                      uint32_t flags);

/****************************************************************************
 * Name:  esp32c6_cpuint_initialize
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

int esp32c6_cpuint_initialize(void);

/****************************************************************************
 * Name:  esp32c6_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   priority - Interrupt's priority (1 - 5).
 *   type     - Interrupt's type (level or edge).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp32c6_setup_irq(int periphid, int priority, int type);

/****************************************************************************
 * Name:  esp32c6_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32c6_setup_irq.
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

void esp32c6_teardown_irq(int periphid, int cpuint);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C6_ESP32C6_IRQ_H */
