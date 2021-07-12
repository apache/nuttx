/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_cpuint.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CPUINT_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CPUINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* No peripheral assigned to this CPU interrupt */

#define CPUINT_UNASSIGNED 0xff

/* A low priority definition to be used by drivers */

#define ESP32S2_INT_PRIO_DEF        1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

extern uint8_t g_cpu0_intmap[ESP32S2_NCPUINTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s2_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts.
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
 * Name:  esp32s2_alloc_levelint
 *
 * Description:
 *   Allocate a level CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated level-sensitive, CPU interrupt number is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all level-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32s2_alloc_levelint(int priority);

/****************************************************************************
 * Name:  esp32s2_alloc_edgeint
 *
 * Description:
 *   Allocate an edge CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated edge-sensitive, CPU interrupt number is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all edge-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32s2_alloc_edgeint(int priority);

/****************************************************************************
 * Name:  esp32s2_free_cpuint
 *
 * Description:
 *   Free a previously allocated CPU interrupt by making it available in the
 *   g_cpu0_freeints.
 *
 * Input Parameters:
 *   cpuint         - The CPU interrupt number to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_free_cpuint(int cpuint);

/****************************************************************************
 * Name:  esp32s2_attach_peripheral
 *
 * Description:
 *   Attach a peripheral interrupt to a CPU interrupt.
 *   This function may be called after esp32s2_alloc_edgeint or
 *   esp32s2_alloc_levelint
 *
 * Input Parameters:
 *   periphid       - The peripheral number from irq.h to be assigned to
 *                    a CPU interrupt.
 *   cpuint         - The CPU interrupt to receive the peripheral interrupt
 *                    assignment. This value is returned by
 *                    esp32s2_alloc_edgeint or esp32s2_alloc_levelint.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_attach_peripheral(int periphid, int cpuint);

/****************************************************************************
 * Name:  esp32s2_detach_peripheral
 *
 * Description:
 *   Detach a peripheral interrupt from a CPU interrupt.
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

void esp32s2_detach_peripheral(int periphid, int cpuint);

#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CPUINT_H */
