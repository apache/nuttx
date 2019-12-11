/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_prci.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FE310_HFROSCCFG (FE310_PRCI_BASE + 0x00)
#define FE310_HFXOSCCFG (FE310_PRCI_BASE + 0x04)
#define FE310_PLLCFG    (FE310_PRCI_BASE + 0x08)
#define FE310_PLLOUTDIV (FE310_PRCI_BASE + 0x0c)

#define HFXOSCCFG_HFXOSCEN  (0x1 << 30)
#define HFXOSCCFG_HFXOSCRDY (0x1 << 31)

#define PLLCFG_PLLSEL       (0x1 << 16)
#define PLLCFG_PLLREFSEL    (0x1 << 17)
#define PLLCFG_PLLBYPASS    (0x1 << 18)
#define PLLCFG_PLLLOCK      (0x1 << 31)

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PRCI_H */
