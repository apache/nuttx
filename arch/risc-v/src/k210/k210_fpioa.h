/****************************************************************************
 * arch/risc-v/src/k210/k210_fpioa.h
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

#ifndef __ARCH_RISCV_SRC_K210_K210_FPIOA_H
#define __ARCH_RISCV_SRC_K210_K210_FPIOA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_IO_NUMBER 48

#define K210_IO_FUNC_UARTHS_RX 18  /* UART High speed Receiver */
#define K210_IO_FUNC_UARTHS_TX 19  /* UART High speed Transmitter */
#define K210_IO_FUNC_GPIOHS0   24  /* GPIO High speed 0 */
#define K210_IO_FUNC_GPIOHS1   25  /* GPIO High speed 1 */
#define K210_IO_FUNC_GPIOHS2   26  /* GPIO High speed 2 */
#define K210_IO_FUNC_GPIOHS3   27  /* GPIO High speed 3 */
#define K210_IO_FUNC_GPIOHS4   28  /* GPIO High speed 4 */
#define K210_IO_FUNC_GPIOHS5   29  /* GPIO High speed 5 */
#define K210_IO_FUNC_GPIOHS6   30  /* GPIO High speed 6 */
#define K210_IO_FUNC_GPIOHS7   31  /* GPIO High speed 7 */
#define K210_IO_FUNC_GPIOHS8   32  /* GPIO High speed 8 */
#define K210_IO_FUNC_GPIOHS9   33  /* GPIO High speed 9 */
#define K210_IO_FUNC_GPIOHS10  34  /* GPIO High speed 10 */
#define K210_IO_FUNC_GPIOHS11  35  /* GPIO High speed 11 */
#define K210_IO_FUNC_GPIOHS12  36  /* GPIO High speed 12 */
#define K210_IO_FUNC_GPIOHS13  37  /* GPIO High speed 13 */
#define K210_IO_FUNC_GPIOHS14  38  /* GPIO High speed 14 */
#define K210_IO_FUNC_GPIOHS15  39  /* GPIO High speed 15 */
#define K210_IO_FUNC_GPIOHS16  40  /* GPIO High speed 16 */
#define K210_IO_FUNC_GPIOHS17  41  /* GPIO High speed 17 */
#define K210_IO_FUNC_GPIOHS18  42  /* GPIO High speed 18 */
#define K210_IO_FUNC_GPIOHS19  43  /* GPIO High speed 19 */
#define K210_IO_FUNC_GPIOHS20  44  /* GPIO High speed 20 */
#define K210_IO_FUNC_GPIOHS21  45  /* GPIO High speed 21 */
#define K210_IO_FUNC_GPIOHS22  46  /* GPIO High speed 22 */
#define K210_IO_FUNC_GPIOHS23  47  /* GPIO High speed 23 */
#define K210_IO_FUNC_GPIOHS24  48  /* GPIO High speed 24 */
#define K210_IO_FUNC_GPIOHS25  49  /* GPIO High speed 25 */
#define K210_IO_FUNC_GPIOHS26  50  /* GPIO High speed 26 */
#define K210_IO_FUNC_GPIOHS27  51  /* GPIO High speed 27 */
#define K210_IO_FUNC_GPIOHS28  52  /* GPIO High speed 28 */
#define K210_IO_FUNC_GPIOHS29  53  /* GPIO High speed 29 */
#define K210_IO_FUNC_GPIOHS30  54  /* GPIO High speed 30 */
#define K210_IO_FUNC_GPIOHS31  55  /* GPIO High speed 31 */

#define K210_IO_DS(x)  (x << 8) /* Driving Selector */

#define K210_IO_OUTPUT_ENABLE  (1 << 12)
#define K210_IO_OUTPUT_INVERT  (1 << 13)
#define K210_IO_INPUT_ENABLE   (1 << 20)
#define K210_IO_INPUT_INVERT   (1 << 21)
#define K210_IO_PULL_DOWN      (1 << 16)
#define K210_IO_PULL_UP        (3 << 16)
#define K210_IO_PULL_UP_STRONG (7 << 16)
#define K210_IO_SL             (1 << 19)
#define K210_IO_ST             (1 << 23)

#define K210_IOFLAG_GPIOHS (K210_IO_DS(0xf) | K210_IO_OUTPUT_ENABLE | \
                            K210_IO_INPUT_ENABLE | K210_IO_ST)

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void k210_fpioa_config(uint32_t io, uint32_t ioflag);

#endif /* __ARCH_RISCV_SRC_K210_K210_FPIOA_H */
