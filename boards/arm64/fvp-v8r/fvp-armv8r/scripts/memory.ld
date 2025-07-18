/****************************************************************************
 * boards/arm/qemu/qemu-armv7r/scripts/memory.ld
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

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
/* Split SRAM between kernel and user spaces */

/****************************************************************************
 *   Memory map
 *   +---------------------+
 *   | Kernel text         |
 *   +---------------------+
 *   | Kernel data         |
 *   +---------------------+
 *   | User text           |
 *   +---------------------+
 *   | User data           |
 *   +---------------------+
 *   | Kernel Heap         |
 *   +---------------------+
 *   | User Heap           |
 *   |                     |
 *   +---------------------+
 ****************************************************************************/

#  define KSRAM_START   CONFIG_RAM_START
#  define KSRAM_END     CONFIG_NUTTX_USERSPACE
#  define KSRAM_SIZE    (KSRAM_END - KSRAM_START)
#  define USRAM_START   CONFIG_NUTTX_USERSPACE
#  define USRAM_END     CONFIG_RAM_END
#  define USRAM_SIZE    (USRAM_END - USRAM_START)
#else
/* Give All RAM to kernel */

/****************************************************************************
 *   Memory map
 *   +---------------------+
 *   | Kernel text         |
 *   +---------------------+
 *   | Kernel data         |
 *   +---------------------+
 *   | Kernel Heap         |
 *   +---------------------+
 *
 ****************************************************************************/

#  define KSRAM_START   CONFIG_RAM_START
#  define KSRAM_END     CONFIG_RAM_END
#  define KSRAM_SIZE    (KSRAM_END - KSRAM_START)
#  define USRAM_START   0
#  define USRAM_END     0
#  define USRAM_SIZE    0
#endif

MEMORY
{
  ksram (rwx) : ORIGIN = KSRAM_START, LENGTH = KSRAM_SIZE
#ifdef CONFIG_BUILD_PROTECTED
  usram (rwx) : ORIGIN = USRAM_START, LENGTH = USRAM_SIZE
#endif
}
