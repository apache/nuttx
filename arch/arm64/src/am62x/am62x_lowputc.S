/****************************************************************************
 * arch/arm64/src/am62x/am62x_lowputc.S
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
 *   Low-level UART output for TI AM62x, used before the C serial driver
 *   is initialised (CONFIG_ARCH_EARLY_PRINT path).
 *
 *   The AM62x console UART is a 16550-compatible peripheral.  The base
 *   address is selected at build time either by the generic 16550 console
 *   configuration or by the legacy AM62X UART selection.  U-Boot
 *   configures it at 115200-8N1 before handing off to NuttX, so
 *   arm64_earlyprintinit is a no-op — we simply inherit U-Boot's config.
 *
 *   Register usage in arm64_lowputc follows the AAPCS64 ABI:
 *     x0   — character to transmit (caller-provided, preserved across call)
 *     x15  — UART base address (scratch, callee-save not required for leaf)
 *     w2   — LSR value (scratch)
 *
 *   All other callee-saved registers are untouched because arm64_lowputc
 *   is a leaf function.
 *
 ***************************************************************************/

#include <nuttx/config.h>
#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "am62x_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* Console UART base address.
 *
 * Prefer the configured serial console device.  Fall back to enabled UARTs
 * only if no *_SERIAL_CONSOLE option is set.
 */

#if defined(CONFIG_16550_UART0_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     CONFIG_16550_UART0_BASE
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02860000
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02850000
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02840000
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02830000
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02820000
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02810000
#elif defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_UART_BASE     0x02800000
#elif defined(CONFIG_AM62X_UART6)
#  define CONSOLE_UART_BASE     0x02860000
#elif defined(CONFIG_AM62X_UART5)
#  define CONSOLE_UART_BASE     0x02850000
#elif defined(CONFIG_AM62X_UART4)
#  define CONSOLE_UART_BASE     0x02840000
#elif defined(CONFIG_AM62X_UART3)
#  define CONSOLE_UART_BASE     0x02830000
#elif defined(CONFIG_AM62X_UART2)
#  define CONSOLE_UART_BASE     0x02820000
#elif defined(CONFIG_AM62X_UART1)
#  define CONSOLE_UART_BASE     0x02810000
#else
#  define CONSOLE_UART_BASE     0x02800000  /* Default: UART0 */
#endif

/* 16550 register offsets */

#define UART_THR_OFF            0x00    /* Transmit Holding Register       */
#define UART_LSR_OFF            0x14    /* Line Status Register            */
#define UART_LSR_THRE           (1 << 5)/* TX Holding Register Empty bit   */

/****************************************************************************
 * Private Macros
 ****************************************************************************/

/* early_uart_ready xb, wt
 *   Spin until the TX holding register is empty.
 *   xb  — register holding the UART base address
 *   wt  — scratch word register for LSR read
 */

.macro early_uart_ready xb, wt
1:
    ldr     \wt, [\xb, #UART_LSR_OFF]   /* Read LSR                        */
    tst     \wt, #UART_LSR_THRE         /* Test THRE bit                   */
    b.eq    1b                          /* Loop while THRE == 0            */
.endm

/* early_uart_transmit xb, wt
 *   Write one byte to the THR.
 *   xb  — register holding the UART base address
 *   wt  — word register containing the byte to transmit
 */

.macro early_uart_transmit xb, wt
    strb    \wt, [\xb, #UART_THR_OFF]   /* Write byte to THR               */
.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_earlyprintinit
 *
 * Description:
 *   Hardware-specific UART initialisation for early print.
 *   On AM62x, U-Boot has already configured UART0 (clocks, baud rate,
 *   pin mux) before the image is loaded.  Nothing to do here.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    ret                                 /* U-Boot already initialised UART */

/****************************************************************************
 * Name: arm64_lowputc
 *
 * Description:
 *   Output one character to UART0 by polling the LSR THRE bit.
 *   This function is safe to call before the MMU is enabled (flat map)
 *   and before the C runtime is up.
 *
 * Input Parameters:
 *   x0 - character to transmit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr     x15, =CONSOLE_UART_BASE     /* Load console UART base into x15 */
    early_uart_ready  x15, w2           /* Wait for TX holding reg empty   */
    early_uart_transmit x15, w0         /* Write character                 */
    ret
