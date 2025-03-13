/****************************************************************************
 * arch/arm64/src/a527/a527_lowputc.S
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

/* Arm64 Wrapper for early printing */

#include <nuttx/config.h>
#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "a527_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#define UART0_BASE_ADDRESS 0x02500000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* 16550 UART initialization */

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    ret

/* Wait for 16550 UART to be ready to transmit.
 * xb: Register that contains the UART Base Address
 * wt: Scratch register number
 */

.macro early_uart_ready xb, wt
1:
    ldrh  \wt, [\xb, #0x14]      /* UART_LSR (Line Status Register) */
    tst   \wt, #0x20             /* Check THRE (TX Holding Register Empty) */
    b.eq  1b                     /* Wait for the UART to be ready (THRE=1) */
.endm

/* 16550 UART transmit character.
 * xb: Register that contains the UART Base Address
 * wt: Register that contains the character to transmit
 */

.macro early_uart_transmit xb, wt
    strb  \wt, [\xb]             /* -> UART_THR (TX Holding Register) */
.endm

/* Print a character on the UART. This function is called by C.
 * x0: Character to print
 */

GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =UART0_BASE_ADDRESS
    early_uart_ready    x15, w2
    early_uart_transmit x15, w0
    ret
