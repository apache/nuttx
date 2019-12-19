/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_gpio.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FE310_GPIO_INPUT_VAL   (FE310_GPIO_BASE + 0x00)
#define FE310_GPIO_INPUT_EN    (FE310_GPIO_BASE + 0x04)
#define FE310_GPIO_OUTPUT_EN   (FE310_GPIO_BASE + 0x08)
#define FE310_GPIO_OUTPUT_VAL  (FE310_GPIO_BASE + 0x0c)
#define FE310_GPIO_PU_EN       (FE310_GPIO_BASE + 0x10)
#define FE310_GPIO_DS          (FE310_GPIO_BASE + 0x14)
#define FE310_GPIO_RISE_IE     (FE310_GPIO_BASE + 0x18)
#define FE310_GPIO_RISE_IP     (FE310_GPIO_BASE + 0x1c)
#define FE310_GPIO_FALL_IE     (FE310_GPIO_BASE + 0x20)
#define FE310_GPIO_FALL_IP     (FE310_GPIO_BASE + 0x24)
#define FE310_GPIO_HIGH_IE     (FE310_GPIO_BASE + 0x28)
#define FE310_GPIO_HIGH_IP     (FE310_GPIO_BASE + 0x2c)
#define FE310_GPIO_LOW_IE      (FE310_GPIO_BASE + 0x30)
#define FE310_GPIO_LOW_IP      (FE310_GPIO_BASE + 0x34)
#define FE310_GPIO_IOF_EN      (FE310_GPIO_BASE + 0x38)
#define FE310_GPIO_IOF_SEL     (FE310_GPIO_BASE + 0x3c)
#define FE310_GPIO_OUTPUT_XOR  (FE310_GPIO_BASE + 0x40)

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H */
