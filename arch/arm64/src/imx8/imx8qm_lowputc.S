/****************************************************************************
 * arch/arm64/src/imx8/imx8_lowputs.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "imx8qm_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* 32-bit register definition for imx8qm uart*/
#define UART1_BASE_ADDRESS 0x5a060000

#define UARTSTAT          (0x14)
#define UARTDATA          (0x1C)
#define UARTSTAT_TDRE     (1 << 23)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* PL011 UART initialization */

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    /* Already initialized in bootloader */
    ret

/* i.MX8QM wait LPUART to be ready to transmit
 * rb: register which contains the UART base address
 * rc: scratch register
 */
.macro early_uart_ready xb, c
1:
    ldr   w\c, [\xb, #UARTSTAT]   /* <- Flag register */
    tst   w\c, #UARTSTAT_TDRE     /* Check FIFO EMPTY bit */
    beq   1b                      /* Wait for the UART to be ready */
.endm

/* i.MX8QM LPUART transmit character
 * rb: register which contains the UART base address
 * rt: register which contains the character to transmit */
.macro early_uart_transmit xb, wt
    str   \wt, [\xb, #UARTDATA]  /* -> Data Register */
.endm

/*
 * Print a character on the UART - this function is called by C
 * x0: character to print
 */
GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =UART1_BASE_ADDRESS
    early_uart_ready x15, 2
    early_uart_transmit x15, w0
    ret
