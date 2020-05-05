/****************************************************************************
 * arch/risc-v/src/k210/k210_memorymap.h
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H
#define _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/k210_memorymap.h"
#include "hardware/k210_uart.h"
#include "hardware/k210_clint.h"
#include "hardware/k210_plic.h"
#include "hardware/k210_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define K210_IDLESTACK_BASE  (uintptr_t)&_ebss
#else
#define K210_IDLESTACK_BASE  _ebss
#endif

#define K210_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~7)

#define K210_IDLESTACK0_TOP  (K210_IDLESTACK_BASE + K210_IDLESTACK_SIZE)
#define K210_IDLESTACK1_TOP  (K210_IDLESTACK0_TOP + K210_IDLESTACK_SIZE)

#define K210_IDLESTACK_TOP   (K210_IDLESTACK1_TOP)

#endif /* _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H */
