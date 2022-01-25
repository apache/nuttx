/****************************************************************************
 * arch/arm/src/kl/kl_lowputc.h
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

#ifndef __ARCH_ARM_SRC_KL_KL_LOWPUTC_H
#define __ARCH_ARM_SRC_KL_KL_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "kl_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kl_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization including setup of the console UART.
 *   This UART done early so that the serial console is available for
 *   debugging very early in the boot sequence.
 *
 ****************************************************************************/

void kl_lowsetup(void);

/****************************************************************************
 * Name: kl_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: kl_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void kl_lowputc(uint32_t ch);
#endif

/****************************************************************************
 * Name: kl_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartconfigure(uintptr_t uart_base, uint32_t baud, uint32_t clock,
                      unsigned int parity, unsigned int nbits);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KL_KL_LOWPUTC_H */
