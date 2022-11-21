/****************************************************************************
 * arch/arm64/src/a64/a64_lowputc.S
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
 *    DESCRIPTION
 *       Low-level console output for Allwinner A64
 *
 ***************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "a64_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

 /* Allwinner A64 UART0 Base Address */

 #define UART0_BASE_ADDRESS 0x1C28000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize A64 UART
 * xb: Register that contains the UART Base Address
 * c:  Scratch register number
 */

GTEXT(up_earlyserialinit)
SECTION_FUNC(text, up_earlyserialinit)
    ret  /* Do nothing because U-Boot has already initialized UART */

/* Wait for A64 UART to be ready to transmit
 * xb: Register that contains the UART Base Address
 * wt: Scratch register number
 */

.macro early_uart_ready xb, wt
1:
    ldrh  \wt, [\xb, #0x14]      /* UART_LSR (Line Status Register) */
    tst   \wt, #0x20             /* Check THRE (TX Holding Register Empty) */
    b.eq  1b                     /* Wait for the UART to be ready (THRE=1) */
.endm

/* Transmit character to A64 UART
 * xb: Register that contains the UART base address
 * wt: Register that contains the character to transmit
 */

.macro early_uart_transmit xb, wt
    strb  \wt, [\xb]             /* UART_THR (Transmit Holding Register) */
.endm

/* Print a character on the UART - this function is called by C
 * x0: Character to print
 */

GTEXT(up_lowputc)
SECTION_FUNC(text, up_lowputc)
    ldr   x15, =UART0_BASE_ADDRESS
    early_uart_ready x15, w2
    early_uart_transmit x15, w0
    ret
