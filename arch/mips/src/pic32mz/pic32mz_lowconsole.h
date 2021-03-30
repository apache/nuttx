/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_lowconsole.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_LOWCONSOLE_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_LOWCONSOLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_consoleinit
 *
 * Description:
 *   Performs low level initialization of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void pic32mz_consoleinit(void);
#else
#  define pic32mz_consoleinit()
#endif

/****************************************************************************
 * Name: pic32mz_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mz_uartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: pic32mz_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mz_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                           unsigned int parity, unsigned int nbits,
                           bool stop2);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_LOWCONSOLE_H */
