/****************************************************************************
 * arch/arm64/src/fvp-v8r/fvp_lowputc.S
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
#include "serial_pl011.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "fvp_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

 /* 32-bit register definition for qemu pl011 uart */

#define EARLY_UART_PL011_BAUD_RATE  115200

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* PL011 UART initialization
 * xb: register which contains the UART base address
 * c: scratch register number
 */

GTEXT(up_earlyserialinit)
SECTION_FUNC(text, up_earlyserialinit)
    ldr   x15, =CONFIG_UART0_BASE
    mov   x0, #(7372800 / EARLY_UART_PL011_BAUD_RATE % 16)
    strh  w0, [x15, #0x28]      /* -> UARTFBRD (Baud divisor fraction) */
    mov   x0, #(7372800 / EARLY_UART_PL011_BAUD_RATE / 16)
    strh  w0, [x15, #0x24]      /* -> UARTIBRD (Baud divisor integer) */
    mov   x0, #0x60             /* 8n1 */
    str   w0, [x15, #0x2C]      /* -> UARTLCR_H (Line control) */
    ldr   x0, =0x00000301       /* RXE | TXE | UARTEN */
    str   w0, [x15, #0x30]      /* -> UARTCR (Control Register) */
    ret

/* PL011 UART wait UART to be ready to transmit
 * xb: register which contains the UART base address
 * c: scratch register number
 */

.macro early_uart_ready xb, wt
1:
    ldrh  \wt, [\xb, #0x18]      /* <- UARTFR (Flag register) */
    tst   \wt, #0x8              /* Check BUSY bit */
    b.ne  1b                     /* Wait for the UART to be ready */
.endm

/* PL011 UART transmit character
 * xb: register which contains the UART base address
 * wt: register which contains the character to transmit
 */

.macro early_uart_transmit xb, wt
    strb  \wt, [\xb]             /* -> UARTDR (Data Register) */
.endm

/* Print a character on the UART - this function is called by C
 * x0: character to print
 */

GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =CONFIG_UART0_BASE
    early_uart_ready x15, w2
    early_uart_transmit x15, w0
    ret
