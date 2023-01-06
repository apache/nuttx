/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_memorymap.h
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

#ifndef _ARCH_RISCV_SRC_ESP32C6_ESP32C6_MEMORYMAP_H
#define _ARCH_RISCV_SRC_ESP32C6_ESP32C6_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define ESP32C6_IDLESTACK_BASE  (uint32_t)&g_idlestack
#else
#define ESP32C6_IDLESTACK_BASE  g_idlestack
#endif

#define ESP32C6_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~3)
#define ESP32C6_IDLESTACK_TOP  (ESP32C6_IDLESTACK_BASE + ESP32C6_IDLESTACK_SIZE)

#endif /* _ARCH_RISCV_SRC_ESP32C6_ESP32C6_MEMORYMAP_H */
