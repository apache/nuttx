/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_plic.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_PLIC_PRIORITY    (K210_PLIC_BASE + 0x000000)
#define K210_PLIC_PENDING1    (K210_PLIC_BASE + 0x001000)
#define K210_PLIC_ENABLE1     (K210_PLIC_BASE + 0x002000)
#define K210_PLIC_ENABLE2     (K210_PLIC_BASE + 0x002004)
#define K210_PLIC_THRESHOLD   (K210_PLIC_BASE + 0x200000)
#define K210_PLIC_CLAIM       (K210_PLIC_BASE + 0x200004)

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H */
