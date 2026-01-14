/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_lowputc.S
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
 *    DESCRIPTION
 *       Low-level console output for XilinX Zynq Mpsoc
 *
 ***************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "zynq_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

 /* XilinX Zynq Mpsoc XCZU28DR UART0 Base Address */
#define UART0_BASE_ADDRESS   0xff000000
#define XUARTPS_SR_OFFSET    0x002CU /**< Channel Status [14:0] */
#define XUARTPS_SR_TXFULL    0x0010U /**< TX FIFO full */
#define XUARTPS_FIFO_OFFSET  0x0030U /**< FIFO [7:0] */
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize Zynq Mpsoc UART
 * xb: Register that contains the UART Base Address
 * c:  Scratch register number
 */

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    ret  /* Do nothing because U-Boot has already initialized UART */

/* Wait for XCZU28DR UART to be ready to transmit
 * xb: Register that contains the UART Base Address
 * wt: Scratch register number
 */

.macro early_uart_ready xb, wt
1:
    ldr   \wt, [\xb, #XUARTPS_SR_OFFSET]    /* UART_LSR (Line Status Register) */
    tst   \wt, #XUARTPS_SR_TXFULL           /* Check THRE (TX Holding Register Empty) */
    b.ne  1b                                /* Wait for the UART to be ready (THRE=1) */
.endm

/* Transmit character to XCZU28DR UART
 * xb: Register that contains the UART base address
 * wt: Register that contains the character to transmit
 */

.macro early_uart_transmit xb, wt
    str   \wt, [\xb, #XUARTPS_FIFO_OFFSET]  /* -> Data Register */
.endm

/* Print a character on the UART - this function is called by C
 * x0: Character to print
 */

GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =UART0_BASE_ADDRESS
    early_uart_ready x15, w2
    early_uart_transmit x15, w0
    ret
