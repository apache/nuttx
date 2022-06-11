/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_irq.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_IRQ_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_INT_LEVEL           (0 << 0)
#define ESP32C3_INT_EDGE            (1 << 0)

#define ESP32C3_INT_PRIO_MIN        1
#define ESP32C3_INT_PRIO_MAX        7

#define ESP32C3_INT_PRIO_DEF        1

/****************************************************************************
 * Name: esp32c3_bind_irq
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

void esp32c3_bind_irq(uint8_t cpuint, uint8_t periphid, uint8_t prio,
                      uint32_t flags);

/****************************************************************************
 * Name: esp32c3_request_irq
 *
 * Description:
 *   Request IRQ and resource with given parameters.
 *
 * Input Parameters:
 *   periphid  - Peripheral ID
 *   prio  - Interrupt priority
 *   flags - Interrupt flags
 *
 * Returned Value:
 *   Allocated CPU interrupt on success, a negated error on failure.
 *
 ****************************************************************************/

int esp32c3_request_irq(uint8_t periphid, uint8_t prio, uint32_t flags);

/****************************************************************************
 * Name: esp32c3_free_cpuint
 *
 * Description:
 *   Free IRQ and resource.
 *
 * Input Parameters:
 *   periphid - Peripheral ID.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_free_cpuint(uint8_t periphid);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_IRQ_H */
