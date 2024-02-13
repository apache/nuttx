/****************************************************************************
 * arch/arm64/src/imx9/imx93_lowputs.S
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
 *       Wrapper for early printk
 *
 ***************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"

#include "hardware/imx9_lpuart.h"
#include "hardware/imx93/imx93_memorymap.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "imx93_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* PL011 UART initialization */

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    /* TODO: Assumes u-boot has set us up, assumption is fine for now */
    ret

/* i.MX93 wait LPUART to be ready to transmit
 * rb: register which contains the UART base address
 * rc: scratch register
 */
.macro early_uart_ready rb, rc
1:
    ldr   \rc, [\rb, #UARTSTAT]   /* <- Flag register */
    tst   \rc, #UARTSTAT_TDRE     /* Check FIFO EMPTY bit */
    beq   1b                      /* Wait for the UART to be ready */
.endm

/* i.MX93 LPUART transmit character
 * rb: register which contains the UART base address
 * rt: register which contains the character to transmit */
.macro early_uart_transmit rb, rt
    str   \rt, [\rb, #UARTDATA]  /* -> Data Register */
.endm

/*
 * Print a character on the UART - this function is called by C
 * w0: character to print
 */
GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =IMX9_LPUART1_BASE
    early_uart_ready x15, w2
    early_uart_transmit x15, w0
    ret
