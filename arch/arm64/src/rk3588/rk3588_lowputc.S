/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_lowputc.S
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
 ****************************************************************************
 *
 * DESCRIPTION
 *   Early UART output for RK3588 / RK3588S.
 *
 *   The Orange Pi 5 Pro debug UART is UART2, a 16550-compatible block
 *   with 4-byte register spacing.  Boot firmware already leaves it in a
 *   usable state, so early printk only needs to poll THRE and write bytes.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"

    .file    "rk3588_lowputc.S"

#define CONSOLE_UART_BASE   CONFIG_16550_UART0_BASE
#define UART_THR_OFF        0x00
#define UART_LSR_OFF        0x14
#define UART_LSR_THRE       (1 << 5)

.macro early_uart_ready xb, wt
1:
    ldr     \wt, [\xb, #UART_LSR_OFF]
    tst     \wt, #UART_LSR_THRE
    b.eq    1b
.endm

.macro early_uart_transmit xb, wt
    str     \wt, [\xb, #UART_THR_OFF]
.endm

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    ret

GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr     x15, =CONSOLE_UART_BASE
    early_uart_ready x15, w2
    early_uart_transmit x15, w0
    ret
