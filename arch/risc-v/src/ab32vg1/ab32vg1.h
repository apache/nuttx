/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_AB32VG1_H
#define __ARCH_RISCV_SRC_AB32VG1_AB32VG1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "riscv_common_memorymap.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include "riscv_internal.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The idle thread stack starts at _ebss */

#ifndef __ASSEMBLY__
#  define AB32VG1_IDLESTACK_BASE  (uintptr_t)_ebss
#else
#  define AB32VG1_IDLESTACK_BASE  _ebss
#endif

#define AB32VG1_IDLESTACK_TOP     (AB32VG1_IDLESTACK_BASE + SMP_STACK_SIZE)

/* Symbols that the board linker script must provide, in addition to the
 * common ones (_stext, _sbss, _ebss, ...): the end of the image in SRAM and
 * the end of the code placed in AECRAM.
 */

#ifndef __ASSEMBLY__
extern uint8_t _esram[];
extern uint8_t _eaecram[];
#endif

/* System clock */

#define AB32VG1_SYSCLK_FREQUENCY  (CONFIG_AB32VG1_SYSCLK_MHZ * 1000000)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_clockconfig
 *
 * Description:
 *   Switch the system clock to PLL0 / n (CONFIG_AB32VG1_SYSCLK_MHZ).
 *
 ****************************************************************************/

void ab32vg1_clockconfig(void);

/****************************************************************************
 * Name: ab32vg1_lowsetup
 *
 * Description:
 *   Configure the console UART for polled output, early in the boot.
 *
 ****************************************************************************/

void ab32vg1_lowsetup(void);

/****************************************************************************
 * Name: ab32vg1_boardinitialize
 *
 * Description:
 *   Board-specific early initialization, provided by the board logic.
 *
 ****************************************************************************/

void ab32vg1_boardinitialize(void);

/****************************************************************************
 * Name: ab32vg1_dispatch_trap
 *
 * Description:
 *   Called from the trap entry code with the trap kind (0 exception,
 *   1 interrupt, 2 system call) and the saved registers.  Returns the
 *   register frame to restore.
 *
 ****************************************************************************/

uintreg_t *ab32vg1_dispatch_trap(int kind, uintreg_t *regs);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_AB32VG1_AB32VG1_H */
