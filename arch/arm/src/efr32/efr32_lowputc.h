/****************************************************************************
 * arch/arm/src/efr32/efr32_lowputc.h
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

#ifndef __ARCH_ARM_SRC_EFR32_EFR32_LOWPUTC_H
#define __ARCH_ARM_SRC_EFR32_EFR32_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: efr32_lowsetup
 *
 * Description:
 *   Called early in the boot sequence, after efr32_clockconfig(), to
 *   enable the USART clocks, route the board's pins and configure the
 *   serial console so that debug output is available.
 *
 ****************************************************************************/

void efr32_lowsetup(void);

/****************************************************************************
 * Name: efr32_usart_configure
 *
 * Description:
 *   Configure a USART for asynchronous operation (baud rate, data bits,
 *   parity and stop bits) and enable its receiver and transmitter.  The
 *   USART must be disabled or idle.
 *
 * Input Parameters:
 *   base     - USART base address
 *   baud     - Baud rate
 *   parity   - 0 = none, 1 = odd, 2 = even
 *   bits     - Data bits (4 to 9)
 *   stop2    - True for two stop bits
 *
 ****************************************************************************/

void efr32_usart_configure(uintptr_t base, uint32_t baud,
                           unsigned int parity, unsigned int bits,
                           bool stop2);

/****************************************************************************
 * Name: efr32_usart_reset
 *
 * Description:
 *   Disable a USART and return its registers to the reset state.
 *
 ****************************************************************************/

void efr32_usart_reset(uintptr_t base);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EFR32_EFR32_LOWPUTC_H */
